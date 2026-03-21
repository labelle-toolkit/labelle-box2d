const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    // iOS SDK path — passed by the consumer's build.zig for cross-compilation.
    const ios_sdk_path = b.option([]const u8, "ios_sdk_path", "iOS SDK path for cross-compilation");

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

    // iOS cross-compilation: add SDK system include paths so box2d C code
    // and @cImport can find math.h, mach/mach_time.h, etc.
    if (ios_sdk_path) |sdk| {
        const include_path: std.Build.LazyPath = .{ .cwd_relative = b.pathJoin(&.{ sdk, "usr/include" }) };
        box2d_lib.root_module.addSystemIncludePath(include_path);
        mod.addSystemIncludePath(include_path);
    }

    // labelle-core: injected by the assembler at build time via addImport.
    // When building standalone (tests, remote fetch), use the lazy dependency.
    if (b.lazyDependency("labelle_core", .{ .target = target, .optimize = optimize })) |core_dep| {
        mod.addImport("labelle-core", core_dep.module("labelle-core"));
    }
}
