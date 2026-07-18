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
    .radius = 15, // pixels — matches the 15px visual circle
    .restitution = 0.5,
});

// Static ground
const ground = g.createEntity();
g.setPosition(ground, .{ .x = 0, .y = -270 });
g.ecs_backend.addComponent(ground, box2d.PhysicsBody{ .body_type = .static });
g.ecs_backend.addComponent(ground, box2d.PhysicsCollider{
    .shape_type = .box,
    .width = 800,
    .height = 50,
});
```

Or use prefabs:

```zon
// prefabs/ball.zon
.{
    .components = .{
        .Shape = .{ .shape = .{ .circle = .{ .radius = 15 } }, .color = .{ .r = 255, .g = 100, .b = 50, .a = 255 } },
        .RigidBody = .{ .gravity_scale = 1.0 },
        .Collider = .{ .shape_type = .circle, .radius = 15, .restitution = 0.5 },
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

### Shapes

`shape_type` selects one of five shapes; each reads its own `Collider` fields (all pixels, all relative to the body position):

| Shape | Fields | Notes |
|-------|--------|-------|
| `.box` | `width`, `height` | Anchored per `anchor` (top-left by default, see below) |
| `.diamond` | `width`, `height` | Rotated-square polygon; anchors like `.box` |
| `.circle` | `radius` | Always centre-anchored |
| `.segment` | `segment_x1`, `segment_y1`, `segment_x2`, `segment_y2` | Line between two endpoints; **two-sided** (collides from both sides). Effectively for `.static` bodies — a segment has zero area, so on a dynamic body box2d falls back to mass=1 with no rotational inertia and `density` is ignored |
| `.capsule` | `capsule_x1`, `capsule_y1`, `capsule_x2`, `capsule_y2`, `radius` | Stadium swept by `radius` around the spine `(capsule_x1, capsule_y1) → (capsule_x2, capsule_y2)` — the canonical character collider (slides over surfaces without catching box corners). `radius` must be > 0; coincident endpoints silently degrade to a circle |

```zig
// One-way-looking floor (physics is still two-sided)
g.ecs_backend.addComponent(floor, box2d.PhysicsCollider{
    .shape_type = .segment,
    .segment_x1 = -400, .segment_y1 = 0,
    .segment_x2 = 400,  .segment_y2 = 0,
});

// Character capsule: vertical spine, 50px tall + 2×15px caps
g.ecs_backend.addComponent(player, box2d.PhysicsCollider{
    .shape_type = .capsule,
    .capsule_x1 = 0, .capsule_y1 = -25,
    .capsule_x2 = 0, .capsule_y2 = 25,
    .radius = 15,
});
```

Segment/capsule geometry is authored relative to the body position, so the box/diamond `anchor` setting does not apply to them — but the shared `offset_x`/`offset_y` fields still do: the runtime adds them to every endpoint before conversion.

### Units & anchoring

Collider `width`/`height`/`radius`/`offset` are in **pixels** (matching `Position` and visual `Shape`), converted to box2d meters internally via `ppm`.

Box/diamond colliders anchor `Position` at the shape's **top-left corner** by default (`anchor = .top_left`), matching how the renderer draws rectangles — the body is offset by half the dimensions so visual and physics coincide, including under rotation. For sprite-backed entities (sprite pivot defaults to centre), set `anchor = .center`. Circles are always centre-anchored. Anchor and collider dimensions are captured when the body is first synced — recreate the body if they change afterward.

## Collision Detection

### Polling (Touching component)

```zig
if (g.ecs_backend.getComponent(entity, box2d.PhysicsTouching)) |touching| {
    for (touching.slice()) |other| {
        // entity is touching other right now
    }
}
```

### Events

The plugin emits collision/sensor notifications through the engine's
`PluginEvents` union (RFC-PLUGIN-EVENTS). Subscribe with a hook-handler
struct on the merged `PluginEvents` union, or consume them from a flow
via `OnEvent`:

| Event | Payload |
|-------|---------|
| `box2d__collision_begin` | `entity_a: u32, entity_b: u32` |
| `box2d__collision_end` | `entity_a: u32, entity_b: u32` |
| `box2d__collision_hit` | `entity_a, entity_b: u32, point_x, point_y, normal_x, normal_y, speed: f32` |
| `box2d__sensor_enter` | `sensor_entity: u32, visitor_entity: u32` |
| `box2d__sensor_exit` | `sensor_entity: u32, visitor_entity: u32` |

> The legacy `on_collision_*` / `on_sensor_*` raw-slot callbacks were
> removed in 0.5.0 (RFC-PLUGIN-EVENTS phase 6). Use the events above.

## Sensors

Trigger volumes that detect overlap without collision response:

```zig
g.ecs_backend.addComponent(entity, box2d.PhysicsCollider{
    .shape_type = .circle,
    .radius = 100, // pixels
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
- All positions AND collider dimensions in pixels, auto-converted to/from meters via `ppm`
- Coordinate system matches the engine (Y-up, origin at screen center)

## Requirements

- Zig 0.15.2+
- LaBelle CLI v1.13.0+ (SystemRegistry support)
- A real ECS backend (`.ecs = .zig_ecs` in project.labelle)

## License

MIT
