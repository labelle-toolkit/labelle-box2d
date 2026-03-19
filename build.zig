const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const box2d_dep = b.dependency("box2d_c", .{
        .target = target,
        .optimize = optimize,
    });

    const box2d_lib = box2d_dep.artifact("box2d");

    // Plugin module — exports Components and Systems for the engine
    const mod = b.addModule("labelle_box2d", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });
    mod.addIncludePath(box2d_dep.path("include"));
    mod.linkLibrary(box2d_lib);

    // labelle-core: injected by the assembler at build time via addImport.
    // When building standalone (tests, remote fetch), use the lazy dependency.
    if (b.lazyDependency("labelle_core", .{ .target = target, .optimize = optimize })) |core_dep| {
        mod.addImport("labelle-core", core_dep.module("labelle-core"));
    }
}
