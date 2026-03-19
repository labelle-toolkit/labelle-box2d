# labelle-box2d

Box2D v3 physics plugin for [LaBelle](https://labelle.games). Pure ECS — add components to entities and physics just works.

## Quick Start

Add to your `project.labelle`:

```zon
.plugins = .{
    .{ .name = "box2d", .repo = "local:../labelle-box2d" },
},
```

Create entities with physics components:

```zig
const box2d = @import("box2d");

// Dynamic ball
const ball = g.createEntity();
g.setPosition(ball, .{ .x = 0, .y = 200 });
g.ecs_backend.addComponent(ball, box2d.PhysicsBody{});
g.ecs_backend.addComponent(ball, box2d.PhysicsCollider{
    .shape_type = .circle,
    .radius = 0.3,
    .restitution = 0.5,
});

// Static ground
const ground = g.createEntity();
g.setPosition(ground, .{ .x = 0, .y = -270 });
g.ecs_backend.addComponent(ground, box2d.PhysicsBody{ .body_type = .static });
g.ecs_backend.addComponent(ground, box2d.PhysicsCollider{
    .shape_type = .box,
    .width = 16.0,
    .height = 1.0,
});
```

Or use prefabs:

```zon
// prefabs/ball.zon
.{
    .components = .{
        .Shape = .{ .shape = .{ .circle = .{ .radius = 15 } }, .color = .{ .r = 255, .g = 100, .b = 50, .a = 255 } },
        .RigidBody = .{ .gravity_scale = 1.0 },
        .Collider = .{ .shape_type = .circle, .radius = 0.3, .restitution = 0.5 },
    },
}
```

No script needed for physics — the plugin's `Systems` handle everything automatically.

## Components

| Component | Purpose |
|-----------|---------|
| `RigidBody` | Body type, gravity, damping, bullet mode |
| `Collider` | Shape, size, density, friction, restitution, filtering |
| `Touching` | Auto-populated: entities currently in contact |
| `Sensor` | Auto-populated: entities inside a trigger volume |

All components are auto-discovered by the engine's `ComponentRegistryWithPlugins`.

## Collision Detection

### Polling (Touching component)

```zig
if (g.ecs_backend.getComponent(entity, box2d.PhysicsTouching)) |touching| {
    for (touching.slice()) |other| {
        // entity is touching other right now
    }
}
```

### Callbacks

```zig
pub fn setup(g: anytype) void {
    box2d.on_collision_begin = onHit;
    box2d.on_collision_hit = onImpact;
}

fn onHit(entity_a: u32, entity_b: u32) void {
    // contact started
}

fn onImpact(a: u32, b: u32, px: f32, py: f32, nx: f32, ny: f32, speed: f32) void {
    // high-speed impact — speed is approach velocity in m/s
}
```

Available callbacks: `on_collision_begin`, `on_collision_end`, `on_collision_hit`, `on_sensor_enter`, `on_sensor_exit`.

## Sensors

Trigger volumes that detect overlap without collision response:

```zig
g.ecs_backend.addComponent(entity, box2d.PhysicsCollider{
    .shape_type = .circle,
    .radius = 2.0,
    .is_sensor = true, // trigger volume, no collision
});
// PhysicsSensor component is auto-added
```

## Joints

```zig
const box2d = @import("box2d");

// Spring between two entities
_ = box2d.createDistanceJoint(body_a, body_b, .{ .stiffness = 5.0, .damping = 0.7 });

// Hinge with motor
_ = box2d.createRevoluteJoint(body_a, body_b, pivot_x, pivot_y, .{
    .enable_motor = true,
    .motor_speed = 3.14,
    .max_motor_torque = 100,
});

// Slider
_ = box2d.createPrismaticJoint(body_a, body_b, anchor_x, anchor_y, 1, 0, .{
    .enable_limit = true,
    .lower_translation = -100,
    .upper_translation = 100,
});

// Rigid weld
_ = box2d.createWeldJoint(body_a, body_b, anchor_x, anchor_y, .{});
```

## Body Operations

```zig
box2d.applyForce(body, 500, 0);          // continuous force (pixels/s²)
box2d.applyImpulse(body, 0, 200);        // instant impulse (pixels/s)
box2d.applyTorque(body, 50);             // rotational force
box2d.setVelocity(body, 100, 0);         // set velocity directly
const vel = box2d.getVelocity(body);     // [2]f32 in pixels/s
box2d.setBodyPosition(body, 0, 300);     // teleport
box2d.setAngle(body, 3.14 / 4);         // set rotation
const mass = box2d.getMass(body);        // kg
```

## Ray Casting

```zig
const result = box2d.rayCast(origin_x, origin_y, target_x, target_y);
if (result.hit) {
    // result.entity — what was hit
    // result.point_x, result.point_y — hit location
    // result.normal_x, result.normal_y — surface normal
    // result.fraction — 0..1 along the ray
}
```

## Collision Filtering

```zig
const PLAYER: u64 = 0x0001;
const ENEMY: u64  = 0x0002;
const BULLET: u64 = 0x0004;

// Player collides with enemies and bullets
g.ecs_backend.addComponent(entity, box2d.PhysicsCollider{
    .category_bits = PLAYER,
    .mask_bits = ENEMY | BULLET,
});

// Bullets don't collide with each other
g.ecs_backend.addComponent(bullet, box2d.PhysicsCollider{
    .category_bits = BULLET,
    .mask_bits = PLAYER | ENEMY, // not BULLET
});
```

## Configuration

```zig
box2d.ppm = 50.0;                    // pixels per meter (default: 50)
box2d.show_collision_gizmos = true;   // debug arrows on collisions
box2d.setGravity(0, -500);           // change gravity at runtime
```

## Debug Gizmos

When `show_collision_gizmos` is enabled:
- **Green arrows** — contact begin (body A → B)
- **Red arrows** — hit impacts (length = approach speed)
- **Yellow arrows** — sensor enter (sensor → visitor)

## Architecture

- **No labelle-core dependency** in build.zig — injected by the assembler
- **`pub const Components`** — auto-discovered by `ComponentRegistryWithPlugins`
- **`pub const Systems`** — auto-dispatched by `SystemRegistry`
- **Box2D v3.1** via [allyourcodebase/box2d](https://github.com/allyourcodebase/box2d)
- All positions in pixels, auto-converted to/from meters via `ppm`
- Coordinate system matches the engine (Y-up, origin at screen center)

## Requirements

- Zig 0.15.2+
- LaBelle CLI v1.13.0+ (SystemRegistry support)
- A real ECS backend (`.ecs = .zig_ecs` in project.labelle)

## License

MIT
