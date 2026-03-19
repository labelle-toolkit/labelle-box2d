const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const box2d_dep = b.dependency("box2d_c", .{
        .target = target,
        .optimize = optimize,
    });

    const box2d_lib = box2d_dep.artifact("box2d");

    // Plugin module — exports Components for auto-discovery by ComponentRegistryWithPlugins
    const mod = b.addModule("labelle_box2d", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });
    // Provide box2d headers for @cImport and link the library
    mod.addIncludePath(box2d_dep.path("include"));
    mod.linkLibrary(box2d_lib);
}
