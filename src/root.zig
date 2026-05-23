//! labelle-box2d — Box2D physics plugin for LaBelle.
//!
//! Pure ECS plugin: exports Components (auto-discovered) and Systems
//! (auto-dispatched by SystemRegistry). Games just add RigidBody + Collider
//! components to entities — physics runs automatically.
//!
//! Features:
//!   - RigidBody + Collider components → auto Box2D body creation
//!   - Touching component (polling collision state)
//!   - Callbacks: on_collision_begin, on_collision_end, on_collision_hit
//!   - Sensors (trigger volumes)
//!   - Joints (distance, revolute, prismatic, weld, wheel, mouse)
//!   - Ray/shape casting
//!   - Body operations (force, impulse, velocity, teleport)
//!   - Collision filtering (category/mask bits)
//!   - Debug gizmos

const core = @import("labelle-core");
const Position = core.Position;

pub const b2 = @cImport({
    @cInclude("box2d/box2d.h");
});

// ── ECS Components (auto-discovered by ComponentRegistryWithPlugins) ──

pub const Components = struct {
    pub const RigidBody = PhysicsBody;
    pub const Collider = PhysicsCollider;
    pub const Touching = PhysicsTouching;
    pub const Sensor = PhysicsSensor;
};

// ── Systems (auto-dispatched by SystemRegistry) ──

pub const Systems = struct {
    pub fn setup(game: anytype) void {
        _ = game;
        var world_def = b2.b2DefaultWorldDef();
        world_def.gravity = .{ .x = 0, .y = -10.0 };
        world_id = b2.b2CreateWorld(&world_def);
        initialized = true;
    }

    pub fn tick(game: anytype, dt: f32) void {
        if (!initialized) return;
        syncNewBodies(game);
        b2.b2World_Step(world_id, dt, 4);
    }

    pub fn postTick(game: anytype, _: f32) void {
        if (!initialized) return;
        syncPositionsBack(game);
        processContacts(game);
        processSensorEvents(game);
    }

    pub fn deinit() void {
        if (initialized) {
            b2.b2DestroyWorld(world_id);
            initialized = false;
        }
    }
};

// ── Module state ───────────────────────────────────────────

var world_id: b2.b2WorldId = undefined;
var initialized: bool = false;

/// Gizmo categories exported by this plugin.
/// Auto-discovered by the debug inspector at comptime.
pub const GizmoCategories = struct {
    pub const Collision: u8 = 1;
    pub const Physics: u8 = 2;
    pub const Sensor: u8 = 3;
};

// ── Events (auto-discovered by assembler PluginEvents codegen) ──
//
// RFC-PLUGIN-EVENTS phase 1 (assembler `602aebd`) folds every plugin
// module's `pub const Events` into a `PluginEvents` tagged union with
// variant tag `<plugin>__<event>` (e.g. `box2d__collision_begin`).
// Phase 2 (this commit) is the matching plugin emit side — see the
// dual-emit `game.emit(...)` calls in `processContacts` /
// `processSensorEvents` below. Migration phase 1: the legacy raw
// `pub var on_collision_*` callback slots stay live alongside the
// new buffered events for one release.
//
// Payload field shapes mirror the v1 callback signatures at
// `root.zig:90-95` byte-for-byte so a flow consuming the new event
// gets the same data as a v1 callback subscriber.
pub const Events = struct {
    /// Two entities started touching.
    pub const collision_begin = struct {
        entity_a: u32,
        entity_b: u32,
    };
    /// Two entities stopped touching.
    pub const collision_end = struct {
        entity_a: u32,
        entity_b: u32,
    };
    /// Solver-reported hit event (large impact); carries contact point,
    /// surface normal, and approach speed.
    pub const collision_hit = struct {
        entity_a: u32,
        entity_b: u32,
        point_x: f32,
        point_y: f32,
        normal_x: f32,
        normal_y: f32,
        speed: f32,
    };
    /// A visitor entity entered a sensor trigger volume.
    pub const sensor_enter = struct {
        sensor_entity: u32,
        visitor_entity: u32,
    };
    /// A visitor entity exited a sensor trigger volume.
    pub const sensor_exit = struct {
        sensor_entity: u32,
        visitor_entity: u32,
    };
};

// ── FlowNodes (RFC-FLOW-VOCABULARY phase 5) ────────────────────
//
// Palette-ready verbs the flow editor surfaces under a "box2d"
// category. Each decl is a `core.flow.FlowNode(.{ .impl = … })`
// factory call: reflection on `impl` produces pin names, types, and
// the command/reporter kind; `.pins` overrides labels where the
// generic param name doesn't read well.
//
// Discovered by the labelle-assembler plugin walk (parallel phase 2,
// `labelle-assembler#177`); consumed by the flow-codegen `CustomNode`
// lowering (later phase) and the labelle-gui palette (phase 4). Until
// those land, this block is *additive* — no runtime behaviour change
// for existing games.
//
// **First-param convention** (RFC §1): every `impl` takes
// `game: anytype` first. flow-codegen threads the game pointer in at
// codegen time; the remaining params become input pins. This is why
// the wrappers below look up the `PhysicsBody` component from the
// entity rather than taking a `*const PhysicsBody` directly the way
// the underlying Zig-level helpers do — flows reason in entity IDs.
pub const FlowNodes = struct {
    pub const apply_impulse = core.flow.FlowNode(.{
        .impl = flowApplyImpulse,
        .docs = "Apply a linear impulse to the body's center of mass (pixels/s).",
        .pins = .{
            .ix = .{ .label = "Impulse X" },
            .iy = .{ .label = "Impulse Y" },
        },
    });

    pub const apply_force = core.flow.FlowNode(.{
        .impl = flowApplyForce,
        .docs = "Apply a force to the body's center of mass (pixels/s²).",
        .pins = .{
            .fx = .{ .label = "Force X" },
            .fy = .{ .label = "Force Y" },
        },
    });

    pub const apply_torque = core.flow.FlowNode(.{
        .impl = flowApplyTorque,
        .docs = "Apply a rotational torque to the body.",
    });

    pub const set_velocity = core.flow.FlowNode(.{
        .impl = flowSetVelocity,
        .docs = "Set the body's linear velocity directly (pixels/s).",
        .pins = .{
            .vx = .{ .label = "Velocity X" },
            .vy = .{ .label = "Velocity Y" },
        },
    });

    pub const get_velocity = core.flow.FlowNode(.{
        .impl = flowGetVelocity,
        .docs = "Read the body's linear velocity (pixels/s).",
    });

    pub const get_angular_velocity = core.flow.FlowNode(.{
        .impl = flowGetAngularVelocity,
        .docs = "Read the body's angular velocity (radians/s).",
    });

    pub const get_position = core.flow.FlowNode(.{
        .impl = flowGetPosition,
        .docs = "Read the entity's world-space position (pixels).",
    });

    pub const set_position = core.flow.FlowNode(.{
        .impl = flowSetPosition,
        .docs = "Teleport the body to a new world-space position (pixels).",
    });

    pub const get_angle = core.flow.FlowNode(.{
        .impl = flowGetAngle,
        .docs = "Read the body's rotation angle (radians).",
    });

    pub const set_angle = core.flow.FlowNode(.{
        .impl = flowSetAngle,
        .docs = "Set the body's rotation angle (radians).",
    });

    pub const get_mass = core.flow.FlowNode(.{
        .impl = flowGetMass,
        .docs = "Read the body's mass (kg).",
    });

    pub const ray_cast = core.flow.FlowNode(.{
        .impl = flowRayCast,
        .docs = "Cast a ray from origin to target (pixels). Returns the closest hit.",
        .pins = .{
            .origin_x = .{ .label = "Origin X" },
            .origin_y = .{ .label = "Origin Y" },
            .target_x = .{ .label = "Target X" },
            .target_y = .{ .label = "Target Y" },
        },
    });

    pub const body_at = core.flow.FlowNode(.{
        .impl = flowBodyAt,
        .docs = "Return the entity whose body contains the given world-space point, or 0 if none.",
    });

    pub const set_gravity = core.flow.FlowNode(.{
        .impl = flowSetGravity,
        .docs = "Set world gravity (pixels/s²).",
        .pins = .{
            .gx = .{ .label = "Gravity X" },
            .gy = .{ .label = "Gravity Y" },
        },
    });
};

// ── PinStyles (RFC-FLOW-VOCABULARY phase 5) ────────────────────
//
// Display metadata for the nominal types this plugin exposes on flow
// pins. Primitives (`u32`, `f32`, `bool`, …) are already covered by
// `core.flow.default_pin_styles`; the entries below only style
// box2d-specific types the editor would otherwise render with no
// distinct color.
//
// Conventionally keyed by *type name* — the assembler walks
// `@TypeOf(PinStyles)`'s decls and maps each decl name back to the
// matching plugin-exported type at discovery time (phase 2).
pub const PinStyles = struct {
    /// `JointId` — cyan, distinct from the entity-id yellow + the
    /// integer-blue of plain numeric pins. Joints are sticky handles
    /// returned by `createDistanceJoint` / `createRevoluteJoint` /
    /// etc. that the game holds across frames.
    pub const JointId = core.flow.PinStyle{
        .label = "Joint",
        .color = core.flow.Color{ .r = 80, .g = 200, .b = 200, .a = 255 },
    };

    /// `RayResult` — magenta, marks "physics query result" payloads
    /// (currently just `rayCast`'s return value) as a distinct shape
    /// from generic structs. The struct itself must be wired (RFC §2),
    /// so the color here is purely a visual aid for the wire.
    pub const RayResult = core.flow.PinStyle{
        .label = "Ray Result",
        .color = core.flow.Color{ .r = 198, .g = 100, .b = 196, .a = 255 },
    };
};

// ── Flow node impls — game-threading wrappers around the
//     entity-and-component-facing API above. Kept private; the
//     `FlowNodes` factory holds them as comptime decls.
//
// Each takes `game: anytype` first per RFC §1, then a plain `u32`
// entity ID (or world-space coordinates for the world-query verbs).
// The wrapper resolves the entity → `*PhysicsBody` lookup and
// delegates to the existing low-level helpers (`applyImpulse`,
// `setVelocity`, …). A missing body component is a silent no-op —
// flows that lose the body mid-frame shouldn't crash the game.

/// Output struct for the `get_velocity` flow node. Two-component
/// vector with explicit field names — flow-codegen unpacks the
/// fields into named output pins so downstream nodes can wire `x`
/// and `y` independently.
pub const Vec2Out = struct { x: f32, y: f32 };

fn flowApplyImpulse(game: anytype, entity: u32, ix: f32, iy: f32) void {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        applyImpulse(body, ix, iy);
    }
}

fn flowApplyForce(game: anytype, entity: u32, fx: f32, fy: f32) void {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        applyForce(body, fx, fy);
    }
}

fn flowApplyTorque(game: anytype, entity: u32, torque: f32) void {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        applyTorque(body, torque);
    }
}

fn flowSetVelocity(game: anytype, entity: u32, vx: f32, vy: f32) void {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        setVelocity(body, vx, vy);
    }
}

fn flowGetVelocity(game: anytype, entity: u32) Vec2Out {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        const v = getVelocity(body);
        return .{ .x = v[0], .y = v[1] };
    }
    return .{ .x = 0, .y = 0 };
}

fn flowGetAngularVelocity(game: anytype, entity: u32) f32 {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        return getAngularVelocity(body);
    }
    return 0;
}

fn flowGetPosition(game: anytype, entity: u32) Vec2Out {
    if (game.ecs_backend.getComponent(entity, Position)) |pos| {
        return .{ .x = pos.x, .y = pos.y };
    }
    return .{ .x = 0, .y = 0 };
}

fn flowSetPosition(game: anytype, entity: u32, x: f32, y: f32) void {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        setBodyPosition(body, x, y);
    }
    // Keep the ECS-side Position in sync for the same frame so
    // gameplay code that reads it after the flow runs sees the new
    // value; postTick's syncPositionsBack will overwrite next frame
    // anyway, but the immediate read is what most flows expect.
    if (game.ecs_backend.getComponent(entity, Position)) |pos| {
        pos.x = x;
        pos.y = y;
    }
}

fn flowGetAngle(game: anytype, entity: u32) f32 {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        return getAngle(body);
    }
    return 0;
}

fn flowSetAngle(game: anytype, entity: u32, angle: f32) void {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        setAngle(body, angle);
    }
}

fn flowGetMass(game: anytype, entity: u32) f32 {
    if (game.ecs_backend.getComponent(entity, PhysicsBody)) |body| {
        return getMass(body);
    }
    return 0;
}

fn flowRayCast(game: anytype, origin_x: f32, origin_y: f32, target_x: f32, target_y: f32) RayResult {
    _ = game;
    return rayCast(origin_x, origin_y, target_x, target_y);
}

/// `body_at` — minimal point query implemented as a short horizontal
/// ray cast across the queried point. Box2D's proper point-overlap
/// API (`b2World_OverlapAABB`) requires a C callback + context
/// pointer; the ray-cast trick is a one-liner that hits any body
/// wider than 1 pixel and is good enough for the canonical "what did
/// the player click on?" use case. A dedicated overlap-query wrapper
/// is a follow-up if a flow author hits the limit.
fn flowBodyAt(game: anytype, x: f32, y: f32) u32 {
    _ = game;
    const result = rayCast(x - 0.5, y, x + 0.5, y);
    return if (result.hit) result.entity else 0;
}

fn flowSetGravity(game: anytype, gx: f32, gy: f32) void {
    _ = game;
    setGravity(gx, gy);
}

// Convenience aliases
pub const GIZMO_COLLISION: u8 = GizmoCategories.Collision;
pub const GIZMO_PHYSICS: u8 = GizmoCategories.Physics;
pub const GIZMO_SENSOR: u8 = GizmoCategories.Sensor;

/// Pixels-per-meter conversion factor.
pub var ppm: f32 = 50.0;
/// Show debug gizmo arrows on collisions.
pub var show_collision_gizmos: bool = true;

// ── DEPRECATED — raw-slot callbacks ───────────────────────────
//
// **DEPRECATED — see RFC-PLUGIN-EVENTS migration. Removed in next
// release.**
//
// The `pub var on_collision_*` / `on_sensor_*` slots below are the v1
// notification mechanism: a hand-written game would assign a
// `?*const fn(...)` and the plugin would call it from
// `processContacts` / `processSensorEvents`. RFC-PLUGIN-EVENTS phase 2
// added `pub const Events` (above) and dual-emits through
// `game.emit(.{ .box2d__... = ... })` alongside these slots so a v1
// subscriber kept working through the migration window. Phase 6 (this
// commit) converted every in-tree flow to the new `name` form
// (flow-codegen `1182a80` + bouncing-ball `8a3b4c5`); no code path
// inside the toolkit reads these slots anymore.
//
// **Removal plan.** A follow-up release drops these `pub var`s and the
// matching `if (cb) |…| cb(...);` call sites in
// `processContacts` / `processSensorEvents` below — the dual-emit
// collapses to a single `game.emit`. New code MUST subscribe via a
// hook-handler struct on the merged `PluginEvents` union (the same way
// flow-codegen's new-form `OnEvent` emits the `FlowEventHandler`
// struct).

/// Collision callbacks.
pub var on_collision_begin: ?*const fn (entity_a: u32, entity_b: u32) void = null;
pub var on_collision_end: ?*const fn (entity_a: u32, entity_b: u32) void = null;
pub var on_collision_hit: ?*const fn (entity_a: u32, entity_b: u32, point_x: f32, point_y: f32, normal_x: f32, normal_y: f32, speed: f32) void = null;
/// Sensor callbacks — trigger volumes.
pub var on_sensor_enter: ?*const fn (sensor_entity: u32, visitor_entity: u32) void = null;
pub var on_sensor_exit: ?*const fn (sensor_entity: u32, visitor_entity: u32) void = null;

// ══════════════════════════════════════════════════════════════
// Components
// ══════════════════════════════════════════════════════════════

pub const BodyType = enum { static, kinematic, dynamic };

/// Rigid body component.
pub const PhysicsBody = struct {
    body_type: BodyType = .dynamic,
    gravity_scale: f32 = 1.0,
    linear_damping: f32 = 0.0,
    angular_damping: f32 = 0.0,
    fixed_rotation: bool = false,
    bullet: bool = false,
    _body_id: b2.b2BodyId = std.mem.zeroes(b2.b2BodyId),
    _synced: bool = false,
};

pub const ShapeType = enum { box, circle };

/// Collider component with collision filtering.
pub const PhysicsCollider = struct {
    shape_type: ShapeType = .box,
    width: f32 = 1.0,
    height: f32 = 1.0,
    radius: f32 = 0.5,
    density: f32 = 1.0,
    friction: f32 = 0.3,
    restitution: f32 = 0.0,
    is_sensor: bool = false,
    offset_x: f32 = 0,
    offset_y: f32 = 0,
    /// Collision filtering — category this shape belongs to.
    category_bits: u64 = 0x0001,
    /// Collision filtering — categories this shape collides with.
    mask_bits: u64 = 0xFFFFFFFFFFFFFFFF,
    /// Collision filtering — group index (negative = never collide within group).
    group_index: i32 = 0,
};

/// Sensor component — trigger volume, no collision response.
/// Add alongside RigidBody + Collider (with is_sensor=true).
pub const PhysicsSensor = struct {
    /// Entities currently inside this sensor.
    visitors: [MAX_TOUCHING]u32 = std.mem.zeroes([MAX_TOUCHING]u32),
    count: u8 = 0,

    pub fn contains(self: *const PhysicsSensor, entity: u32) bool {
        for (self.visitors[0..self.count]) |e| {
            if (e == entity) return true;
        }
        return false;
    }

    pub fn slice(self: *const PhysicsSensor) []const u32 {
        return self.visitors[0..self.count];
    }

    pub fn add(self: *PhysicsSensor, entity: u32) void {
        if (self.count >= MAX_TOUCHING) return;
        for (self.visitors[0..self.count]) |e| {
            if (e == entity) return;
        }
        self.visitors[self.count] = entity;
        self.count += 1;
    }

    pub fn remove(self: *PhysicsSensor, entity: u32) void {
        for (0..self.count) |i| {
            if (self.visitors[i] == entity) {
                if (i < self.count - 1) {
                    self.visitors[i] = self.visitors[self.count - 1];
                }
                self.count -= 1;
                return;
            }
        }
    }
};

pub const MAX_TOUCHING: usize = 8;

/// Queryable collision state (auto-managed by physics system).
pub const PhysicsTouching = struct {
    entities: [MAX_TOUCHING]u32 = std.mem.zeroes([MAX_TOUCHING]u32),
    count: u8 = 0,

    pub fn contains(self: *const PhysicsTouching, entity: u32) bool {
        for (self.entities[0..self.count]) |e| {
            if (e == entity) return true;
        }
        return false;
    }

    pub fn slice(self: *const PhysicsTouching) []const u32 {
        return self.entities[0..self.count];
    }

    pub fn add(self: *PhysicsTouching, entity: u32) void {
        if (self.count >= MAX_TOUCHING) return;
        for (self.entities[0..self.count]) |e| {
            if (e == entity) return;
        }
        self.entities[self.count] = entity;
        self.count += 1;
    }

    pub fn remove(self: *PhysicsTouching, entity: u32) void {
        for (0..self.count) |i| {
            if (self.entities[i] == entity) {
                if (i < self.count - 1) {
                    self.entities[i] = self.entities[self.count - 1];
                }
                self.count -= 1;
                return;
            }
        }
    }
};

// ══════════════════════════════════════════════════════════════
// Joints
// ══════════════════════════════════════════════════════════════

pub const JointId = b2.b2JointId;

/// Create a distance joint (spring) between two entities.
pub fn createDistanceJoint(body_a: *const PhysicsBody, body_b: *const PhysicsBody, opts: struct {
    length: f32 = 0, // 0 = auto from current positions
    min_length: f32 = 0,
    max_length: f32 = 0,
    stiffness: f32 = 0,
    damping: f32 = 0,
    collide_connected: bool = false,
}) JointId {
    var def = b2.b2DefaultDistanceJointDef();
    def.bodyIdA = body_a._body_id;
    def.bodyIdB = body_b._body_id;
    if (opts.length > 0) def.length = opts.length / ppm;
    if (opts.min_length > 0) def.minLength = opts.min_length / ppm;
    if (opts.max_length > 0) def.maxLength = opts.max_length / ppm;
    if (opts.stiffness > 0) def.hertz = opts.stiffness;
    if (opts.damping > 0) def.dampingRatio = opts.damping;
    def.collideConnected = opts.collide_connected;
    return b2.b2CreateDistanceJoint(world_id, &def);
}

/// Create a revolute joint (hinge/pivot) between two entities.
pub fn createRevoluteJoint(body_a: *const PhysicsBody, body_b: *const PhysicsBody, anchor_x: f32, anchor_y: f32, opts: struct {
    enable_limit: bool = false,
    lower_angle: f32 = 0,
    upper_angle: f32 = 0,
    enable_motor: bool = false,
    motor_speed: f32 = 0,
    max_motor_torque: f32 = 0,
    collide_connected: bool = false,
}) JointId {
    var def = b2.b2DefaultRevoluteJointDef();
    def.bodyIdA = body_a._body_id;
    def.bodyIdB = body_b._body_id;
    def.localAnchorA = b2.b2Body_GetLocalPoint(body_a._body_id, .{ .x = anchor_x / ppm, .y = anchor_y / ppm });
    def.localAnchorB = b2.b2Body_GetLocalPoint(body_b._body_id, .{ .x = anchor_x / ppm, .y = anchor_y / ppm });
    def.enableLimit = opts.enable_limit;
    def.lowerAngle = opts.lower_angle;
    def.upperAngle = opts.upper_angle;
    def.enableMotor = opts.enable_motor;
    def.motorSpeed = opts.motor_speed;
    def.maxMotorTorque = opts.max_motor_torque;
    def.collideConnected = opts.collide_connected;
    return b2.b2CreateRevoluteJoint(world_id, &def);
}

/// Create a prismatic joint (slider) between two entities.
pub fn createPrismaticJoint(body_a: *const PhysicsBody, body_b: *const PhysicsBody, anchor_x: f32, anchor_y: f32, axis_x: f32, axis_y: f32, opts: struct {
    enable_limit: bool = false,
    lower_translation: f32 = 0,
    upper_translation: f32 = 0,
    enable_motor: bool = false,
    motor_speed: f32 = 0,
    max_motor_force: f32 = 0,
    collide_connected: bool = false,
}) JointId {
    var def = b2.b2DefaultPrismaticJointDef();
    def.bodyIdA = body_a._body_id;
    def.bodyIdB = body_b._body_id;
    def.localAnchorA = b2.b2Body_GetLocalPoint(body_a._body_id, .{ .x = anchor_x / ppm, .y = anchor_y / ppm });
    def.localAnchorB = b2.b2Body_GetLocalPoint(body_b._body_id, .{ .x = anchor_x / ppm, .y = anchor_y / ppm });
    def.localAxisA = .{ .x = axis_x, .y = axis_y };
    def.enableLimit = opts.enable_limit;
    def.lowerTranslation = opts.lower_translation / ppm;
    def.upperTranslation = opts.upper_translation / ppm;
    def.enableMotor = opts.enable_motor;
    def.motorSpeed = opts.motor_speed;
    def.maxMotorForce = opts.max_motor_force;
    def.collideConnected = opts.collide_connected;
    return b2.b2CreatePrismaticJoint(world_id, &def);
}

/// Create a weld joint (rigid connection) between two entities.
pub fn createWeldJoint(body_a: *const PhysicsBody, body_b: *const PhysicsBody, anchor_x: f32, anchor_y: f32, opts: struct {
    stiffness: f32 = 0,
    damping: f32 = 0,
    collide_connected: bool = false,
}) JointId {
    var def = b2.b2DefaultWeldJointDef();
    def.bodyIdA = body_a._body_id;
    def.bodyIdB = body_b._body_id;
    def.localAnchorA = b2.b2Body_GetLocalPoint(body_a._body_id, .{ .x = anchor_x / ppm, .y = anchor_y / ppm });
    def.localAnchorB = b2.b2Body_GetLocalPoint(body_b._body_id, .{ .x = anchor_x / ppm, .y = anchor_y / ppm });
    if (opts.stiffness > 0) def.angularHertz = opts.stiffness;
    if (opts.damping > 0) def.angularDampingRatio = opts.damping;
    def.collideConnected = opts.collide_connected;
    return b2.b2CreateWeldJoint(world_id, &def);
}

/// Destroy a joint.
pub fn destroyJoint(joint_id: JointId) void {
    b2.b2DestroyJoint(joint_id);
}

// ══════════════════════════════════════════════════════════════
// Body operations
// ══════════════════════════════════════════════════════════════

/// Apply a force to the body's center of mass (in pixels/s²).
pub fn applyForce(body: *const PhysicsBody, fx: f32, fy: f32) void {
    if (!body._synced) return;
    b2.b2Body_ApplyForceToCenter(body._body_id, .{ .x = fx / ppm, .y = fy / ppm }, true);
}

/// Apply an impulse to the body's center of mass (in pixels/s).
pub fn applyImpulse(body: *const PhysicsBody, ix: f32, iy: f32) void {
    if (!body._synced) return;
    b2.b2Body_ApplyLinearImpulseToCenter(body._body_id, .{ .x = ix / ppm, .y = iy / ppm }, true);
}

/// Apply torque (rotation force).
pub fn applyTorque(body: *const PhysicsBody, torque: f32) void {
    if (!body._synced) return;
    b2.b2Body_ApplyTorque(body._body_id, torque, true);
}

/// Set linear velocity directly (in pixels/s).
pub fn setVelocity(body: *const PhysicsBody, vx: f32, vy: f32) void {
    if (!body._synced) return;
    b2.b2Body_SetLinearVelocity(body._body_id, .{ .x = vx / ppm, .y = vy / ppm });
}

/// Get linear velocity (in pixels/s).
pub fn getVelocity(body: *const PhysicsBody) [2]f32 {
    if (!body._synced) return .{ 0, 0 };
    const v = b2.b2Body_GetLinearVelocity(body._body_id);
    return .{ v.x * ppm, v.y * ppm };
}

/// Get angular velocity (radians/s).
pub fn getAngularVelocity(body: *const PhysicsBody) f32 {
    if (!body._synced) return 0;
    return b2.b2Body_GetAngularVelocity(body._body_id);
}

/// Teleport body to a new position (in pixels).
pub fn setBodyPosition(body: *const PhysicsBody, x: f32, y: f32) void {
    if (!body._synced) return;
    const rot = b2.b2Body_GetRotation(body._body_id);
    b2.b2Body_SetTransform(body._body_id, .{ .x = x / ppm, .y = y / ppm }, rot);
}

/// Get body rotation angle (radians).
pub fn getAngle(body: *const PhysicsBody) f32 {
    if (!body._synced) return 0;
    return b2.b2Rot_GetAngle(b2.b2Body_GetRotation(body._body_id));
}

/// Set body rotation angle (radians).
pub fn setAngle(body: *const PhysicsBody, angle: f32) void {
    if (!body._synced) return;
    const pos = b2.b2Body_GetPosition(body._body_id);
    b2.b2Body_SetTransform(body._body_id, pos, b2.b2MakeRot(angle));
}

/// Get body mass (kg).
pub fn getMass(body: *const PhysicsBody) f32 {
    if (!body._synced) return 0;
    return b2.b2Body_GetMass(body._body_id);
}

// ══════════════════════════════════════════════════════════════
// Ray casting & world queries
// ══════════════════════════════════════════════════════════════

pub const RayResult = struct {
    hit: bool = false,
    point_x: f32 = 0,
    point_y: f32 = 0,
    normal_x: f32 = 0,
    normal_y: f32 = 0,
    fraction: f32 = 0,
    entity: u32 = 0,
};

/// Cast a ray from origin to target (in pixels). Returns the closest hit.
pub fn rayCast(origin_x: f32, origin_y: f32, target_x: f32, target_y: f32) RayResult {
    if (!initialized) return .{};

    const origin = b2.b2Vec2{ .x = origin_x / ppm, .y = origin_y / ppm };
    const translation = b2.b2Vec2{
        .x = (target_x - origin_x) / ppm,
        .y = (target_y - origin_y) / ppm,
    };

    const filter = b2.b2DefaultQueryFilter();
    const result = b2.b2World_CastRayClosest(world_id, origin, translation, filter);

    if (result.hit) {
        const entity = entityFromShape(result.shapeId) orelse 0;
        return .{
            .hit = true,
            .point_x = result.point.x * ppm,
            .point_y = result.point.y * ppm,
            .normal_x = result.normal.x,
            .normal_y = result.normal.y,
            .fraction = result.fraction,
            .entity = entity,
        };
    }
    return .{};
}

// ══════════════════════════════════════════════════════════════
// Internal: ECS ↔ Box2D sync
// ══════════════════════════════════════════════════════════════

fn syncNewBodies(game: anytype) void {
    var iter = game.ecs_backend.query(.{ PhysicsBody, Position });
    defer iter.deinit(game.ecs_backend.alloc);

    while (iter.next()) |result| {
        const body: *PhysicsBody = result.comp_0;
        if (body._synced) continue;

        const pos: *const Position = result.comp_1;
        const px = pos.x / ppm;
        const py = pos.y / ppm;

        var body_def = b2.b2DefaultBodyDef();
        body_def.type = switch (body.body_type) {
            .static => b2.b2_staticBody,
            .kinematic => b2.b2_kinematicBody,
            .dynamic => b2.b2_dynamicBody,
        };
        body_def.position = .{ .x = px, .y = py };
        body_def.gravityScale = body.gravity_scale;
        body_def.linearDamping = body.linear_damping;
        body_def.angularDamping = body.angular_damping;
        body_def.fixedRotation = body.fixed_rotation;
        body_def.isBullet = body.bullet;
        body_def.userData = @ptrFromInt(@as(usize, result.entity) + 1);

        body._body_id = b2.b2CreateBody(world_id, &body_def);
        body._synced = true;

        if (game.ecs_backend.getComponent(result.entity, PhysicsCollider)) |collider| {
            attachShape(body._body_id, collider);
        }

        if (!game.ecs_backend.hasComponent(result.entity, PhysicsTouching)) {
            game.ecs_backend.addComponent(result.entity, PhysicsTouching{});
        }

        // Auto-add Sensor component for sensor shapes
        if (game.ecs_backend.getComponent(result.entity, PhysicsCollider)) |collider| {
            if (collider.is_sensor and !game.ecs_backend.hasComponent(result.entity, PhysicsSensor)) {
                game.ecs_backend.addComponent(result.entity, PhysicsSensor{});
            }
        }
    }
}

fn syncPositionsBack(game: anytype) void {
    var iter = game.ecs_backend.query(.{ PhysicsBody, Position });
    defer iter.deinit(game.ecs_backend.alloc);

    while (iter.next()) |result| {
        const body: *const PhysicsBody = result.comp_0;
        if (!body._synced) continue;
        if (body.body_type == .static) continue;

        const b2_pos = b2.b2Body_GetPosition(body._body_id);
        const pos: *Position = result.comp_1;
        pos.x = b2_pos.x * ppm;
        pos.y = b2_pos.y * ppm;
        game.renderer.markPositionDirty(result.entity);
    }
}

fn processContacts(game: anytype) void {
    const events = b2.b2World_GetContactEvents(world_id);

    for (0..@intCast(events.beginCount)) |i| {
        const event = events.beginEvents[i];
        const body_a = b2.b2Shape_GetBody(event.shapeIdA);
        const body_b = b2.b2Shape_GetBody(event.shapeIdB);
        const entity_a = entityFromBody(body_a) orelse continue;
        const entity_b = entityFromBody(body_b) orelse continue;

        if (game.ecs_backend.getComponent(entity_a, PhysicsTouching)) |t| t.add(entity_b);
        if (game.ecs_backend.getComponent(entity_b, PhysicsTouching)) |t| t.add(entity_a);

        if (show_collision_gizmos) {
            const pa = b2.b2Body_GetPosition(body_a);
            const pb = b2.b2Body_GetPosition(body_b);
            game.drawGizmoArrowCategory(GIZMO_COLLISION, pa.x * ppm, pa.y * ppm, pb.x * ppm, pb.y * ppm, 0xFF00FF00);
        }

        if (on_collision_begin) |cb| cb(entity_a, entity_b);
        // RFC-PLUGIN-EVENTS phase 2 — dual-emit alongside the legacy slot
        // (Migration phase 1). The qualified tag `box2d__collision_begin`
        // is the variant declared by the assembler-generated
        // `PluginEvents` union from `Events.collision_begin` above.
        game.emit(.{ .box2d__collision_begin = .{
            .entity_a = entity_a,
            .entity_b = entity_b,
        } });
    }

    for (0..@intCast(events.endCount)) |i| {
        const event = events.endEvents[i];
        const entity_a = entityFromBody(b2.b2Shape_GetBody(event.shapeIdA)) orelse continue;
        const entity_b = entityFromBody(b2.b2Shape_GetBody(event.shapeIdB)) orelse continue;

        if (game.ecs_backend.getComponent(entity_a, PhysicsTouching)) |t| t.remove(entity_b);
        if (game.ecs_backend.getComponent(entity_b, PhysicsTouching)) |t| t.remove(entity_a);
        if (on_collision_end) |cb| cb(entity_a, entity_b);
        game.emit(.{ .box2d__collision_end = .{
            .entity_a = entity_a,
            .entity_b = entity_b,
        } });
    }

    for (0..@intCast(events.hitCount)) |i| {
        const event = events.hitEvents[i];
        const entity_a = entityFromBody(b2.b2Shape_GetBody(event.shapeIdA)) orelse continue;
        const entity_b = entityFromBody(b2.b2Shape_GetBody(event.shapeIdB)) orelse continue;

        if (show_collision_gizmos) {
            const hx = event.point.x * ppm;
            const hy = event.point.y * ppm;
            game.drawGizmoArrowCategory(GIZMO_PHYSICS, hx, hy, hx + event.normal.x * event.approachSpeed * 15, hy + event.normal.y * event.approachSpeed * 15, 0xFFFF0000);
        }

        if (on_collision_hit) |cb| cb(entity_a, entity_b, event.point.x * ppm, event.point.y * ppm, event.normal.x, event.normal.y, event.approachSpeed);
        game.emit(.{ .box2d__collision_hit = .{
            .entity_a = entity_a,
            .entity_b = entity_b,
            .point_x = event.point.x * ppm,
            .point_y = event.point.y * ppm,
            .normal_x = event.normal.x,
            .normal_y = event.normal.y,
            .speed = event.approachSpeed,
        } });
    }
}

fn processSensorEvents(game: anytype) void {
    const events = b2.b2World_GetSensorEvents(world_id);

    for (0..@intCast(events.beginCount)) |i| {
        const event = events.beginEvents[i];
        const sensor_entity = entityFromShape(event.sensorShapeId) orelse continue;
        const visitor_entity = entityFromShape(event.visitorShapeId) orelse continue;

        if (game.ecs_backend.getComponent(sensor_entity, PhysicsSensor)) |s| s.add(visitor_entity);

        if (show_collision_gizmos) {
            const sb = b2.b2Shape_GetBody(event.sensorShapeId);
            const vb = b2.b2Shape_GetBody(event.visitorShapeId);
            const sp = b2.b2Body_GetPosition(sb);
            const vp = b2.b2Body_GetPosition(vb);
            game.drawGizmoArrowCategory(GIZMO_SENSOR, sp.x * ppm, sp.y * ppm, vp.x * ppm, vp.y * ppm, 0xFFFFFF00);
        }

        if (on_sensor_enter) |cb| cb(sensor_entity, visitor_entity);
        game.emit(.{ .box2d__sensor_enter = .{
            .sensor_entity = sensor_entity,
            .visitor_entity = visitor_entity,
        } });
    }

    for (0..@intCast(events.endCount)) |i| {
        const event = events.endEvents[i];
        const sensor_entity = entityFromShape(event.sensorShapeId) orelse continue;
        const visitor_entity = entityFromShape(event.visitorShapeId) orelse continue;

        if (game.ecs_backend.getComponent(sensor_entity, PhysicsSensor)) |s| s.remove(visitor_entity);
        if (on_sensor_exit) |cb| cb(sensor_entity, visitor_entity);
        game.emit(.{ .box2d__sensor_exit = .{
            .sensor_entity = sensor_entity,
            .visitor_entity = visitor_entity,
        } });
    }
}

fn entityFromBody(body_id: b2.b2BodyId) ?u32 {
    const ptr = b2.b2Body_GetUserData(body_id);
    if (ptr == null) return null;
    // `userData` holds `entity + 1` (see body creation) so a valid
    // entity 0 is a non-null pointer — distinct from "no userData".
    return @intCast(@intFromPtr(ptr) - 1);
}

fn entityFromShape(shape_id: b2.b2ShapeId) ?u32 {
    return entityFromBody(b2.b2Shape_GetBody(shape_id));
}

fn attachShape(body_id: b2.b2BodyId, collider: *const PhysicsCollider) void {
    var shape_def = b2.b2DefaultShapeDef();
    shape_def.density = collider.density;
    shape_def.material.friction = collider.friction;
    shape_def.material.restitution = collider.restitution;
    shape_def.isSensor = collider.is_sensor;
    shape_def.enableContactEvents = true;
    shape_def.enableHitEvents = true;
    shape_def.enableSensorEvents = collider.is_sensor;
    shape_def.filter = .{
        .categoryBits = collider.category_bits,
        .maskBits = collider.mask_bits,
        .groupIndex = collider.group_index,
    };

    switch (collider.shape_type) {
        .box => {
            const box = b2.b2MakeOffsetBox(collider.width / 2, collider.height / 2, .{ .x = collider.offset_x, .y = collider.offset_y }, b2.b2MakeRot(0));
            _ = b2.b2CreatePolygonShape(body_id, &shape_def, &box);
        },
        .circle => {
            _ = b2.b2CreateCircleShape(body_id, &shape_def, &b2.b2Circle{
                .center = .{ .x = collider.offset_x, .y = collider.offset_y },
                .radius = collider.radius,
            });
        },
    }
}

// ── Public API ─────────────────────────────────────────────

pub fn getWorldId() b2.b2WorldId {
    return world_id;
}

pub fn isInitialized() bool {
    return initialized;
}

/// Set world gravity (in pixels/s²).
pub fn setGravity(gx: f32, gy: f32) void {
    if (!initialized) return;
    b2.b2World_SetGravity(world_id, .{ .x = gx / ppm, .y = gy / ppm });
}

const std = @import("std");

// ══════════════════════════════════════════════════════════════
// Tests
// ══════════════════════════════════════════════════════════════

test "FlowNodes block declares the expected verbs" {
    // Smoke test: every advertised flow node decl exists, carries
    // the `__is_labelle_flow_node` marker the assembler walks for,
    // and exposes a non-`void` `impl` decl with `game: anytype` as
    // its first parameter (RFC-FLOW-VOCABULARY §1).
    const expected_decls = .{
        "apply_impulse", "apply_force", "apply_torque",
        "set_velocity", "get_velocity", "get_angular_velocity",
        "get_position", "set_position",
        "get_angle",    "set_angle",
        "get_mass",
        "ray_cast",     "body_at",
        "set_gravity",
    };
    inline for (expected_decls) |name| {
        if (!@hasDecl(FlowNodes, name)) {
            @compileError("FlowNodes missing expected decl `" ++ name ++ "`");
        }
        const node = @field(FlowNodes, name);
        const Node = @TypeOf(node);
        if (!@hasDecl(Node, "__is_labelle_flow_node")) {
            @compileError("FlowNodes." ++ name ++ " is not a FlowNode value");
        }
        if (!@hasDecl(Node, "impl")) {
            @compileError("FlowNodes." ++ name ++ " missing `impl` decl");
        }
        const ImplFn = @TypeOf(Node.impl);
        const fn_info = @typeInfo(ImplFn).@"fn";
        if (fn_info.params.len < 1) {
            @compileError("FlowNodes." ++ name ++ "'s impl must take `game: anytype` first");
        }
        // First param must be `anytype` — i.e. `params[0].type == null`.
        if (fn_info.params[0].type != null) {
            @compileError("FlowNodes." ++ name ++ "'s first impl param must be `game: anytype`");
        }
    }
}

test "PinStyles block declares the expected box2d types" {
    const expected = .{ "JointId", "RayResult" };
    inline for (expected) |name| {
        if (!@hasDecl(PinStyles, name)) {
            @compileError("PinStyles missing expected decl `" ++ name ++ "`");
        }
        const style = @field(PinStyles, name);
        // Should be a `core.flow.PinStyle` value.
        if (@TypeOf(style) != core.flow.PinStyle) {
            @compileError("PinStyles." ++ name ++ " must be a `core.flow.PinStyle` value");
        }
        // The whole point of declaring a style is to override
        // *something*. Make sure at least one display field is set.
        if (style.label == null and style.color == null and style.icon == null) {
            @compileError("PinStyles." ++ name ++ " sets no display fields");
        }
    }
}
