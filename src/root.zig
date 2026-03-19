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

/// Pixels-per-meter conversion factor.
pub var ppm: f32 = 50.0;
/// Show debug gizmo arrows on collisions.
pub var show_collision_gizmos: bool = true;

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
        const entity = entityFromShape(result.shapeId);
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
        body_def.userData = @ptrFromInt(@as(usize, result.entity));

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
        const entity_a = entityFromBody(body_a);
        const entity_b = entityFromBody(body_b);
        if (entity_a == 0 or entity_b == 0) continue;

        if (game.ecs_backend.getComponent(entity_a, PhysicsTouching)) |t| t.add(entity_b);
        if (game.ecs_backend.getComponent(entity_b, PhysicsTouching)) |t| t.add(entity_a);

        if (show_collision_gizmos) {
            const pa = b2.b2Body_GetPosition(body_a);
            const pb = b2.b2Body_GetPosition(body_b);
            game.drawGizmoArrow(pa.x * ppm, pa.y * ppm, pb.x * ppm, pb.y * ppm, 0xFF00FF00);
        }

        if (on_collision_begin) |cb| cb(entity_a, entity_b);
    }

    for (0..@intCast(events.endCount)) |i| {
        const event = events.endEvents[i];
        const entity_a = entityFromBody(b2.b2Shape_GetBody(event.shapeIdA));
        const entity_b = entityFromBody(b2.b2Shape_GetBody(event.shapeIdB));
        if (entity_a == 0 or entity_b == 0) continue;

        if (game.ecs_backend.getComponent(entity_a, PhysicsTouching)) |t| t.remove(entity_b);
        if (game.ecs_backend.getComponent(entity_b, PhysicsTouching)) |t| t.remove(entity_a);
        if (on_collision_end) |cb| cb(entity_a, entity_b);
    }

    for (0..@intCast(events.hitCount)) |i| {
        const event = events.hitEvents[i];
        const entity_a = entityFromBody(b2.b2Shape_GetBody(event.shapeIdA));
        const entity_b = entityFromBody(b2.b2Shape_GetBody(event.shapeIdB));
        if (entity_a == 0 or entity_b == 0) continue;

        if (show_collision_gizmos) {
            const hx = event.point.x * ppm;
            const hy = event.point.y * ppm;
            game.drawGizmoArrow(hx, hy, hx + event.normal.x * event.approachSpeed * 15, hy + event.normal.y * event.approachSpeed * 15, 0xFFFF0000);
        }

        if (on_collision_hit) |cb| cb(entity_a, entity_b, event.point.x * ppm, event.point.y * ppm, event.normal.x, event.normal.y, event.approachSpeed);
    }
}

fn processSensorEvents(game: anytype) void {
    const events = b2.b2World_GetSensorEvents(world_id);

    for (0..@intCast(events.beginCount)) |i| {
        const event = events.beginEvents[i];
        const sensor_entity = entityFromShape(event.sensorShapeId);
        const visitor_entity = entityFromShape(event.visitorShapeId);
        if (sensor_entity == 0 or visitor_entity == 0) continue;

        if (game.ecs_backend.getComponent(sensor_entity, PhysicsSensor)) |s| s.add(visitor_entity);

        if (show_collision_gizmos) {
            const sb = b2.b2Shape_GetBody(event.sensorShapeId);
            const vb = b2.b2Shape_GetBody(event.visitorShapeId);
            const sp = b2.b2Body_GetPosition(sb);
            const vp = b2.b2Body_GetPosition(vb);
            game.drawGizmoArrow(sp.x * ppm, sp.y * ppm, vp.x * ppm, vp.y * ppm, 0xFFFFFF00); // yellow
        }

        if (on_sensor_enter) |cb| cb(sensor_entity, visitor_entity);
    }

    for (0..@intCast(events.endCount)) |i| {
        const event = events.endEvents[i];
        const sensor_entity = entityFromShape(event.sensorShapeId);
        const visitor_entity = entityFromShape(event.visitorShapeId);
        if (sensor_entity == 0 or visitor_entity == 0) continue;

        if (game.ecs_backend.getComponent(sensor_entity, PhysicsSensor)) |s| s.remove(visitor_entity);
        if (on_sensor_exit) |cb| cb(sensor_entity, visitor_entity);
    }
}

fn entityFromBody(body_id: b2.b2BodyId) u32 {
    const ptr = b2.b2Body_GetUserData(body_id);
    if (ptr == null) return 0;
    return @intCast(@intFromPtr(ptr));
}

fn entityFromShape(shape_id: b2.b2ShapeId) u32 {
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
