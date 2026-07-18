//! labelle-box2d — Box2D physics plugin for LaBelle.
//!
//! Pure ECS plugin: exports Components (auto-discovered) and Systems
//! (auto-dispatched by SystemRegistry). Games just add RigidBody + Collider
//! components to entities — physics runs automatically.
//!
//! Features:
//!   - RigidBody + Collider components → auto Box2D body creation
//!   - Touching component (polling collision state)
//!   - Collision/sensor events (box2d__collision_begin/end/hit, box2d__sensor_enter/exit)
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
// The assembler scans each plugin module for `pub const Events` and
// folds every `pub const <name> = struct` inside it into a
// `PluginEvents` tagged union with variant tag `<plugin>__<event>`
// (e.g. `box2d__collision_begin`). These buffered events — emitted via
// `emitGameEvent(game, "box2d__…", …)` in `processContacts` /
// `processSensorEvents` below — are the plugin's sole notification
// path (the legacy raw callback slots were removed in box2d#9).
//
// **Entity IDs are u32 by contract.** The assembler discovers this
// block by source scan and references these payload types verbatim in
// the generated union, so they cannot be generic over the game's
// entity ID type; u32 is the toolkit-wide entity width.
// (`emitGameEvent` still @intCasts per field: widening coerces
// losslessly; narrowing is safety-checked at runtime.)
//
// **Units.** Contact points are ppm-scaled (screen pixels, matching
// the Position component); normals are unit vectors; `speed` is
// box2d's raw approach speed in m/s — NOT ppm-scaled. The mix is
// inherited from the removed v1 callback signatures and kept for
// schema stability; read the field docs before doing math with it.
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
    /// Solver-reported hit event (large impact).
    pub const collision_hit = struct {
        entity_a: u32,
        entity_b: u32,
        /// Contact point X, ppm-scaled (screen pixels).
        point_x: f32,
        /// Contact point Y, ppm-scaled (screen pixels).
        point_y: f32,
        /// Surface normal X at the contact (unit vector).
        normal_x: f32,
        /// Surface normal Y at the contact (unit vector).
        normal_y: f32,
        /// Approach speed in box2d units (m/s) — NOT ppm-scaled.
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
        // RFC-FLOW-VOCABULARY §1 / O5 — the editor uses this hint to
        // suggest `ray_cast` from the palette when the user creates a
        // `SetVariable` on a `RayResult`-typed variable. `RayResult` is
        // a struct (can't have an inline default widget per §2's
        // "structs must be wired" rule), so a constructor-node hint is
        // the discoverability story.
        .constructs = "labelle_box2d.RayResult",
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
    // Anchor offset (pixels) from the entity Position to the body
    // centre, captured at first sync (see ColliderAnchor). Used on
    // every sync-back and teleport — cached here so neither path
    // needs a collider lookup.
    _anchor_x: f32 = 0,
    _anchor_y: f32 = 0,
};

pub const ShapeType = enum { box, circle, diamond, segment };

/// How a box/diamond collider is anchored to the entity `Position`.
///   top_left (default) — `Position` is the shape's top edge / left
///       corner in world space. The world is y-up: the gfx rectangle
///       renders from the screen-space top-left, which maps to world
///       `y ∈ [P.y − h, P.y]` — so the body is created at
///       `(P.x + width/2, P.y − height/2)` and visual and physics
///       align, including under rotation (both pivot on the same
///       centre).
///   center — `Position` is the shape's centre. Choose this for
///       sprite-backed entities (sprite pivot defaults to centre) and
///       for pre-0.5.0 behaviour: bodies were always centre-anchored.
/// Ignored for `.circle` colliders — circles are centre-anchored on
/// both the gfx and box2d sides, so there is no mismatch to fix.
///
/// The offset is captured into `PhysicsBody._anchor_*` when the body
/// is first synced (shapes are only attached then), so mutating
/// `width`/`height`/`anchor` after body creation has no effect on the
/// anchor — recreate the body to change it.
pub const ColliderAnchor = enum { top_left, center };

/// Collider component with collision filtering.
///
/// **Dimensions are in PIXELS** (matching `Position`, visual `Shape`,
/// and the rest of the API's force/velocity/joint units) — the plugin
/// converts to box2d meters with `ppm` when shapes are attached.
/// Author a 15px visual circle as `.radius = 15`, never `0.3`.
/// (Pre-0.5.0 these fields were raw box2d meters — see migration
/// note in the 0.5.0 release notes.)
pub const PhysicsCollider = struct {
    shape_type: ShapeType = .box,
    /// Box/diamond full width, pixels.
    width: f32 = 50.0,
    /// Box/diamond full height, pixels.
    height: f32 = 50.0,
    /// Circle radius, pixels.
    radius: f32 = 25.0,
    density: f32 = 1.0,
    friction: f32 = 0.3,
    restitution: f32 = 0.0,
    is_sensor: bool = false,
    /// Shape offset from the body position, pixels.
    offset_x: f32 = 0,
    /// Shape offset from the body position, pixels.
    offset_y: f32 = 0,
    /// Anchor of the entity Position relative to the shape — see
    /// ColliderAnchor. Only meaningful for box/diamond.
    anchor: ColliderAnchor = .top_left,
    /// Segment only: first endpoint relative to the body position, pixels.
    segment_x1: f32 = 0,
    /// Segment only: first endpoint relative to the body position, pixels.
    segment_y1: f32 = 0,
    /// Segment only: second endpoint relative to the body position, pixels.
    segment_x2: f32 = 50,
    /// Segment only: second endpoint relative to the body position, pixels.
    segment_y2: f32 = 0,
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

/// Teleport the entity to a new `Position` (in pixels). The body
/// transform receives the anchor offset captured at sync (see
/// ColliderAnchor): for a top-left-anchored box the body's centre
/// lands at `(x + width/2, y − height/2)`, so the next sync-back
/// reports `Position == (x, y)` exactly.
pub fn setBodyPosition(body: *const PhysicsBody, x: f32, y: f32) void {
    if (!body._synced) return;
    const rot = b2.b2Body_GetRotation(body._body_id);
    b2.b2Body_SetTransform(body._body_id, .{ .x = (x + body._anchor_x) / ppm, .y = (y + body._anchor_y) / ppm }, rot);
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

/// Pixel offset from the entity `Position` to the collider's centre,
/// per the anchor contract (see `ColliderAnchor`). Body creation adds
/// it, sync-back subtracts it, so a top-left-anchored box's visual
/// and physics rectangles coincide (and stay coincident when the body
/// rotates — both sides pivot on the same centre).
fn anchorOffsetPx(collider: ?*const PhysicsCollider) struct { x: f32, y: f32 } {
    const c = collider orelse return .{ .x = 0, .y = 0 };
    if (c.anchor == .center) return .{ .x = 0, .y = 0 };
    // Exhaustive on purpose: adding a ShapeType member without deciding
    // its anchoring must fail compile. Explicit-geometry shapes whose
    // fields are authored relative to the body (segments, capsules)
    // belong in the zero branch, NOT here — their width/height fields
    // are unused, so the box expression would offset them by the
    // meaningless defaults.
    return switch (c.shape_type) {
        // Explicit-geometry shapes (segment endpoints are authored
        // relative to the body) and circles anchor at the centre.
        .circle, .segment => .{ .x = 0, .y = 0 },
        // World space is y-up: the gfx rectangle renders from the
        // TOP edge of `Position` downward (screen-space top-left maps
        // to world y ∈ [P.y − h, P.y]), so its centre is
        // (P.x + w/2, P.y − h/2) — the y term is NEGATIVE.
        .box, .diamond => .{ .x = c.width / 2, .y = -c.height / 2 },
    };
}

fn syncNewBodies(game: anytype) void {
    var iter = game.ecs_backend.query(.{ PhysicsBody, Position });
    defer iter.deinit(game.ecs_backend.alloc);

    while (iter.next()) |result| {
        const body: *PhysicsBody = result.comp_0;
        if (body._synced) continue;

        const pos: *const Position = result.comp_1;
        const collider = game.ecs_backend.getComponent(result.entity, PhysicsCollider);
        const anchor = anchorOffsetPx(collider);
        body._anchor_x = anchor.x;
        body._anchor_y = anchor.y;
        const px = (pos.x + anchor.x) / ppm;
        const py = (pos.y + anchor.y) / ppm;

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

        if (collider) |c| {
            attachShape(body._body_id, c);
        }

        if (!game.ecs_backend.hasComponent(result.entity, PhysicsTouching)) {
            game.ecs_backend.addComponent(result.entity, PhysicsTouching{});
        }

        // Auto-add Sensor component for sensor shapes. RE-FETCH the
        // collider: the addComponent above may have moved component
        // storage on some ECS backends, invalidating the earlier
        // pointer (codex review on #16).
        if (game.ecs_backend.getComponent(result.entity, PhysicsCollider)) |c| {
            if (c.is_sensor and !game.ecs_backend.hasComponent(result.entity, PhysicsSensor)) {
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
        pos.x = b2_pos.x * ppm - body._anchor_x;
        pos.y = b2_pos.y * ppm - body._anchor_y;
        game.renderer.markPositionDirty(result.entity);
    }
}

/// Tolerant plugin-event emit (mirrors `Game.emitEngineEvent` in
/// labelle-engine post-#578). The engine's `Game.emit(event)`
/// requires a concrete `GameEvents` union value, so an anonymous
/// struct literal like `game.emit(.{ .box2d__sensor_enter = … })`
/// no longer coerces — it triggers `error: type 'void' does not
/// support struct initialization syntax` whenever the project's
/// `GameEvents` is `void` (unit tests, or any project the assembler
/// hasn't merged box2d's `Events` into yet).
///
/// This helper does the same comptime gate `emitEngineEvent` uses:
/// when the project's `GameEvents` is a union AND declares the
/// requested variant tag, build the typed payload field-by-field
/// (widening/narrowing ints with @intCast, filling omitted fields
/// from their declared defaults, rejecting payload fields the variant
/// doesn't declare, failing compile on a missing required field)
/// and forward to `game.emit`. Otherwise the entire
/// body folds to a no-op — safe for `GameEvents = void` builds and
/// for projects that haven't enabled box2d events. Both the typed
/// path and every no-op path are unit-tested below
/// ("emitGameEvent …" tests).
inline fn emitGameEvent(game: anytype, comptime tag: []const u8, payload: anytype) void {
    const Game = @TypeOf(game.*);
    const should_emit = comptime blk: {
        if (!@hasDecl(Game, "GameEvents")) break :blk false;
        const GameEvents = Game.GameEvents;
        const ev_info = @typeInfo(GameEvents);
        if (ev_info != .@"union") break :blk false;
        break :blk @hasField(GameEvents, tag);
    };
    if (comptime !should_emit) return;
    const GameEvents = Game.GameEvents;
    const Payload_t = @FieldType(GameEvents, tag);
    var typed: Payload_t = undefined;
    // Reject unknown payload fields up front: the mapping loop below
    // iterates the *target* variant's fields, so a typo'd payload field
    // (`.visitor_entiy = …`) would otherwise be silently dropped — and
    // a defaulted target field would mask the loss entirely.
    inline for (comptime std.meta.fields(@TypeOf(payload))) |pf| {
        if (comptime !@hasField(Payload_t, pf.name)) {
            @compileError("emitGameEvent: unknown payload field '" ++ pf.name ++ "' for variant '" ++ tag ++ "'");
        }
    }
    const fields = comptime std.meta.fields(Payload_t);
    inline for (fields) |f| {
        if (comptime @hasField(@TypeOf(payload), f.name)) {
            const src_val = @field(payload, f.name);
            const SrcT = @TypeOf(src_val);
            if (comptime @typeInfo(f.type) == .int and @typeInfo(SrcT) == .int) {
                @field(typed, f.name) = @intCast(src_val);
            } else {
                @field(typed, f.name) = src_val;
            }
        } else if (comptime f.default_value_ptr != null) {
            @field(typed, f.name) = @as(*const f.type, @ptrCast(@alignCast(f.default_value_ptr.?))).*;
        } else {
            @compileError("emitGameEvent: missing field '" ++ f.name ++ "' for variant '" ++ tag ++ "'");
        }
    }
    game.emit(@unionInit(GameEvents, tag, typed));
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

        // The qualified tag `box2d__collision_begin` is the variant
        // declared by the assembler-generated `PluginEvents` union from
        // `Events.collision_begin` above. Routed through `emitGameEvent`
        // for the comptime-tag gate engine #578 requires (see helper docs
        // above).
        emitGameEvent(game, "box2d__collision_begin", .{
            .entity_a = entity_a,
            .entity_b = entity_b,
        });
    }

    for (0..@intCast(events.endCount)) |i| {
        const event = events.endEvents[i];
        const entity_a = entityFromBody(b2.b2Shape_GetBody(event.shapeIdA)) orelse continue;
        const entity_b = entityFromBody(b2.b2Shape_GetBody(event.shapeIdB)) orelse continue;

        if (game.ecs_backend.getComponent(entity_a, PhysicsTouching)) |t| t.remove(entity_b);
        if (game.ecs_backend.getComponent(entity_b, PhysicsTouching)) |t| t.remove(entity_a);
        emitGameEvent(game, "box2d__collision_end", .{
            .entity_a = entity_a,
            .entity_b = entity_b,
        });
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

        emitGameEvent(game, "box2d__collision_hit", .{
            .entity_a = entity_a,
            .entity_b = entity_b,
            .point_x = event.point.x * ppm,
            .point_y = event.point.y * ppm,
            .normal_x = event.normal.x,
            .normal_y = event.normal.y,
            .speed = event.approachSpeed,
        });
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

        emitGameEvent(game, "box2d__sensor_enter", .{
            .sensor_entity = sensor_entity,
            .visitor_entity = visitor_entity,
        });
    }

    for (0..@intCast(events.endCount)) |i| {
        const event = events.endEvents[i];
        const sensor_entity = entityFromShape(event.sensorShapeId) orelse continue;
        const visitor_entity = entityFromShape(event.visitorShapeId) orelse continue;

        if (game.ecs_backend.getComponent(sensor_entity, PhysicsSensor)) |s| s.remove(visitor_entity);
        emitGameEvent(game, "box2d__sensor_exit", .{
            .sensor_entity = sensor_entity,
            .visitor_entity = visitor_entity,
        });
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
            // Collider dimensions are authored in PIXELS like the rest
            // of the API (positions, forces, velocities, joints) — this
            // is the one place they cross into box2d meters.
            const box = b2.b2MakeOffsetBox(collider.width / ppm / 2, collider.height / ppm / 2, .{ .x = collider.offset_x / ppm, .y = collider.offset_y / ppm }, b2.b2MakeRot(0));
            _ = b2.b2CreatePolygonShape(body_id, &shape_def, &box);
        },
        .circle => {
            _ = b2.b2CreateCircleShape(body_id, &shape_def, &b2.b2Circle{
                .center = .{ .x = collider.offset_x / ppm, .y = collider.offset_y / ppm },
                .radius = collider.radius / ppm,
            });
        },
        .diamond => {
            // A 4-vertex diamond (rhombus) from width/height — the natural
            // footprint for isometric objects. Vertices: top, right, bottom, left.
            const hw = collider.width / ppm / 2;
            const hh = collider.height / ppm / 2;
            const ox = collider.offset_x / ppm;
            const oy = collider.offset_y / ppm;
            var pts = [_]b2.b2Vec2{
                .{ .x = ox, .y = oy - hh },
                .{ .x = ox + hw, .y = oy },
                .{ .x = ox, .y = oy + hh },
                .{ .x = ox - hw, .y = oy },
            };
            const hull = b2.b2ComputeHull(&pts, 4);
            const poly = b2.b2MakePolygon(&hull, 0);
            _ = b2.b2CreatePolygonShape(body_id, &shape_def, &poly);
        },
        .segment => {
            // A line segment collider — terrain edges, walls, one-off
            // platforms. Endpoints are relative to the body plus the
            // shared offset (same as every other shape). `anchor` does
            // not apply: endpoints are explicit geometry, not a
            // bounding box.
            const ox = collider.offset_x;
            const oy = collider.offset_y;
            _ = b2.b2CreateSegmentShape(body_id, &shape_def, &b2.b2Segment{
                .point1 = .{ .x = (collider.segment_x1 + ox) / ppm, .y = (collider.segment_y1 + oy) / ppm },
                .point2 = .{ .x = (collider.segment_x2 + ox) / ppm, .y = (collider.segment_y2 + oy) / ppm },
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

// ── emitGameEvent tests ──
//
// Synthetic games exercising both branches of the comptime gate:
// the typed emit (field mapping, @intCast widening, default filling)
// and every no-op shape (void / non-union GameEvents, missing tag).
// The @compileError branches — a missing required field and an
// unknown payload field — are compile-time failures by design and
// can't be runtime-tested.

/// A game whose GameEvents union declares the emitted tags.
/// `collision_begin` uses u64 entity fields to prove the @intCast
/// widening; `sensor_enter` carries a defaulted field the plugin's
/// payload omits.
const EmitRecordingGame = struct {
    // `pub` on purpose: real games must export GameEvents for
    // cross-module access — mirror the production contract.
    pub const GameEvents = union(enum) {
        box2d__collision_begin: struct { entity_a: u64, entity_b: u64 },
        box2d__sensor_enter: struct {
            sensor_entity: u32,
            visitor_entity: u32,
            extra: f32 = 99.5,
        },
    };

    last: ?GameEvents = null,
    count: usize = 0,

    pub fn emit(self: *@This(), event: GameEvents) void {
        self.last = event;
        self.count += 1;
    }
};

test "emitGameEvent builds the typed payload and widens int fields" {
    var game = EmitRecordingGame{};
    emitGameEvent(&game, "box2d__collision_begin", .{
        .entity_a = @as(u32, 7),
        .entity_b = @as(u32, 42),
    });
    try std.testing.expectEqual(@as(usize, 1), game.count);
    switch (game.last.?) {
        .box2d__collision_begin => |p| {
            try std.testing.expectEqual(@as(u64, 7), p.entity_a);
            try std.testing.expectEqual(@as(u64, 42), p.entity_b);
        },
        else => return error.TestExpectedCollisionBegin,
    }
}

test "emitGameEvent fills omitted fields from their declared defaults" {
    var game = EmitRecordingGame{};
    emitGameEvent(&game, "box2d__sensor_enter", .{
        .sensor_entity = @as(u32, 3),
        .visitor_entity = @as(u32, 9),
    });
    try std.testing.expectEqual(@as(usize, 1), game.count);
    switch (game.last.?) {
        .box2d__sensor_enter => |p| {
            try std.testing.expectEqual(@as(u32, 3), p.sensor_entity);
            try std.testing.expectEqual(@as(u32, 9), p.visitor_entity);
            try std.testing.expectEqual(@as(f32, 99.5), p.extra);
        },
        else => return error.TestExpectedSensorEnter,
    }
}

test "emitGameEvent no-ops when the union lacks the requested tag" {
    var game = EmitRecordingGame{};
    emitGameEvent(&game, "box2d__collision_end", .{
        .entity_a = @as(u32, 1),
        .entity_b = @as(u32, 2),
    });
    try std.testing.expectEqual(@as(usize, 0), game.count);
    try std.testing.expect(game.last == null);
}

test "emitGameEvent no-ops for void and non-union GameEvents" {
    // Compiling IS the assertion here: if the gate wrongly fired for
    // these shapes, `@FieldType(void, tag)` / the absent typed-union
    // path would be a compile error. There is no meaningful runtime
    // state to observe — the genuinely observable no-op (a missing
    // tag in a real union) is the test above.
    const VoidGame = struct {
        pub const GameEvents = void;
    };
    var void_game = VoidGame{};
    emitGameEvent(&void_game, "box2d__collision_begin", .{
        .entity_a = @as(u32, 1),
        .entity_b = @as(u32, 2),
    });

    const StructGame = struct {
        pub const GameEvents = struct {};
    };
    var struct_game = StructGame{};
    emitGameEvent(&struct_game, "box2d__collision_begin", .{
        .entity_a = @as(u32, 1),
        .entity_b = @as(u32, 2),
    });
}

test "emitGameEvent no-ops when the game has no GameEvents decl" {
    // Same contract: compiling is the assertion.
    const BareGame = struct {};
    var game = BareGame{};
    emitGameEvent(&game, "box2d__collision_begin", .{
        .entity_a = @as(u32, 1),
        .entity_b = @as(u32, 2),
    });
}

// ── Collider pixel-units contract (#3) ──
//
// Dimensions are authored in pixels and converted to box2d meters at
// the attachShape boundary. These tests build a real world and read
// the resulting shape geometry back through the box2d C API.

const TestWorld = struct {
    world: b2.b2WorldId,
    body: b2.b2BodyId,
    saved_ppm: f32,

    fn create() TestWorld {
        var world_def = b2.b2DefaultWorldDef();
        const world = b2.b2CreateWorld(&world_def);
        var body_def = b2.b2DefaultBodyDef();
        const body = b2.b2CreateBody(world, &body_def);
        return .{ .world = world, .body = body, .saved_ppm = ppm };
    }

    fn destroy(self: *TestWorld) void {
        // Restore the global ppm — the conversion tests pin it to 50
        // (Copilot review: shared-global leaks make tests order-
        // dependent), and the live-ppm test mutates it on purpose.
        ppm = self.saved_ppm;
        b2.b2DestroyWorld(self.world);
    }

    fn onlyShape(self: *const TestWorld) !b2.b2ShapeId {
        var shapes: [1]b2.b2ShapeId = undefined;
        // A real test failure, not an assert: in a release-mode test
        // run std.debug.assert compiles out and a missing shape would
        // leave the geometry expectations reading undefined memory.
        try std.testing.expectEqual(@as(c_int, 1), b2.b2Body_GetShapes(self.body, &shapes, 1));
        return shapes[0];
    }
};

fn polyExtents(poly: b2.b2Polygon) struct { w: f32, h: f32, cx: f32, cy: f32 } {
    var min_x: f32 = std.math.floatMax(f32);
    var min_y: f32 = std.math.floatMax(f32);
    var max_x: f32 = -std.math.floatMax(f32);
    var max_y: f32 = -std.math.floatMax(f32);
    const n: usize = @intCast(poly.count);
    for (poly.vertices[0..n]) |v| {
        min_x = @min(min_x, v.x);
        min_y = @min(min_y, v.y);
        max_x = @max(max_x, v.x);
        max_y = @max(max_y, v.y);
    }
    return .{ .w = max_x - min_x, .h = max_y - min_y, .cx = (min_x + max_x) / 2, .cy = (min_y + max_y) / 2 };
}

test "attachShape converts box dimensions from pixels to meters" {
    var tw = TestWorld.create();
    defer tw.destroy();
    ppm = 50;

    // 100×50 px box offset by (10, 20) px; at ppm=50 → 2×1 m at (0.2, 0.4) m.
    attachShape(tw.body, &.{
        .shape_type = .box,
        .width = 100,
        .height = 50,
        .offset_x = 10,
        .offset_y = 20,
    });

    const ext = polyExtents(b2.b2Shape_GetPolygon(try tw.onlyShape()));
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), ext.w, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), ext.h, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), ext.cx, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), ext.cy, 0.001);
}

test "attachShape converts circle radius and center from pixels to meters" {
    var tw = TestWorld.create();
    defer tw.destroy();
    ppm = 50;

    // 25 px radius, center offset (5, -10) px → 0.5 m at (0.1, -0.2) m.
    attachShape(tw.body, &.{
        .shape_type = .circle,
        .radius = 25,
        .offset_x = 5,
        .offset_y = -10,
    });

    const circle = b2.b2Shape_GetCircle(try tw.onlyShape());
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), circle.radius, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), circle.center.x, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.2), circle.center.y, 0.001);
}

test "attachShape converts diamond dimensions from pixels to meters" {
    var tw = TestWorld.create();
    defer tw.destroy();
    ppm = 50;

    // 80×40 px diamond, no offset → 1.6×0.8 m bounding extents.
    attachShape(tw.body, &.{
        .shape_type = .diamond,
        .width = 80,
        .height = 40,
    });

    const ext = polyExtents(b2.b2Shape_GetPolygon(try tw.onlyShape()));
    try std.testing.expectApproxEqAbs(@as(f32, 1.6), ext.w, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), ext.h, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), ext.cx, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), ext.cy, 0.001);
}

test "attachShape conversion follows the live ppm setting" {
    var tw = TestWorld.create();
    defer tw.destroy();

    // A game that re-tunes ppm must get consistently scaled colliders
    // (TestWorld.destroy restores the saved value).
    ppm = 100.0;

    attachShape(tw.body, &.{ .shape_type = .circle, .radius = 25 });

    const circle = b2.b2Shape_GetCircle(try tw.onlyShape());
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), circle.radius, 0.001);
}

// ── Anchor/pivot contract (#4) ──

test "anchorOffsetPx: box and diamond offset by half dimensions" {
    // y-up world: the visual rect extends DOWNWARD from Position, so
    // its centre is (P.x + w/2, P.y − h/2) — negative y term.
    const box_col: PhysicsCollider = .{ .shape_type = .box, .width = 100, .height = 40 };
    const box_off = anchorOffsetPx(&box_col);
    try std.testing.expectEqual(@as(f32, 50), box_off.x);
    try std.testing.expectEqual(@as(f32, -20), box_off.y);

    const dia_col: PhysicsCollider = .{ .shape_type = .diamond, .width = 80, .height = 60 };
    const dia_off = anchorOffsetPx(&dia_col);
    try std.testing.expectEqual(@as(f32, 40), dia_off.x);
    try std.testing.expectEqual(@as(f32, -30), dia_off.y);
}

test "anchorOffsetPx: circle never offsets, whatever the anchor" {
    var circle_col: PhysicsCollider = .{ .shape_type = .circle, .radius = 25, .width = 100, .height = 40 };
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(&circle_col).x);
    circle_col.anchor = .center;
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(&circle_col).x);
}

test "anchorOffsetPx: segment endpoints are explicit geometry — never offset" {
    const seg_col: PhysicsCollider = .{ .shape_type = .segment, .width = 100, .height = 40 };
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(&seg_col).x);
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(&seg_col).y);
}

test "anchorOffsetPx: center anchor and missing collider never offset" {
    const centered: PhysicsCollider = .{ .shape_type = .box, .width = 100, .height = 40, .anchor = .center };
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(&centered).x);
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(&centered).y);
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(null).x);
    try std.testing.expectEqual(@as(f32, 0), anchorOffsetPx(null).y);
}

test "setBodyPosition applies the cached anchor offset" {
    var tw = TestWorld.create();
    defer tw.destroy();

    // Teleport a top-left-anchored 100×50 box (anchor +50, −25) to
    // Position (200, 100): the body centre must land at
    // ((200+50)/50, (100−25)/50) = (5, 1.5) m so sync-back reports
    // the requested Position exactly.
    const body: PhysicsBody = .{
        ._body_id = tw.body,
        ._synced = true,
        ._anchor_x = 50,
        ._anchor_y = -25,
    };
    setBodyPosition(&body, 200, 100);

    const p = b2.b2Body_GetPosition(tw.body);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), p.x, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), p.y, 0.001);
}

test "attachShape converts segment endpoints from pixels to meters (#1)" {
    var tw = TestWorld.create();
    defer tw.destroy();
    ppm = 50;

    // (0, 25) → (150, 25) px wall, offset (10, -5) px; at ppm=50 →
    // (0.2, 0.4) → (3.2, 0.4) m. The shared offset applies to segment
    // endpoints like every other shape.
    attachShape(tw.body, &.{
        .shape_type = .segment,
        .segment_x1 = 0,
        .segment_y1 = 25,
        .segment_x2 = 150,
        .segment_y2 = 25,
        .offset_x = 10,
        .offset_y = -5,
    });

    const seg = b2.b2Shape_GetSegment(try tw.onlyShape());
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), seg.point1.x, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), seg.point1.y, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.2), seg.point2.x, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), seg.point2.y, 0.001);
}
