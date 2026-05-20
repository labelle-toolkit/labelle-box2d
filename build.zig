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

    // WASM cross-compilation: Zig does not ship libc headers for
    // `wasm32-emscripten`, so the box2d C compile cannot find
    // <math.h>/<stdlib.h>/etc. (35x `'math.h' file not found` in
    // box2d/math_functions.h). Those headers live in emscripten's SDK
    // sysroot. Plumb the emsdk sysroot include path into both the
    // box2d C library artifact (where the C sources compile) and the
    // plugin module (which @cImports the box2d headers) — mirrors the
    // iOS `ios_sdk_path` block above and labelle-sokol's emsdk fix.
    // Gated strictly on `.emscripten` so desktop / iOS / Android
    // builds are untouched.
    if (target.result.os.tag == .emscripten) {
        if (b.lazyDependency("emsdk", .{})) |emsdk_dep| {
            const sysroot_include = emsdk_dep.path("upstream/emscripten/cache/sysroot/include");
            box2d_lib.root_module.addSystemIncludePath(sysroot_include);
            mod.addSystemIncludePath(sysroot_include);
        }

        // box2d's core.h maps the WASM CPU branch (`B2_CPU_WASM`) onto
        // `B2_SIMD_SSE2`, which `#include <emmintrin.h>` — x86 SSE2
        // intrinsics. Emscripten only emulates those when the C
        // compile enables wasm SIMD (`-msimd128`); the Zig build of
        // box2d_c uses `-mcpu baseline` for wasm32, so the SSE2
        // intrinsics fail to compile. Force box2d's scalar
        // (`B2_SIMD_NONE`) path — a fully supported box2d
        // configuration — by defining `BOX2D_DISABLE_SIMD`. Define it
        // on both the C library compile AND the plugin module: `mod`
        // (`src/root.zig`) `@cImport`s box2d's headers, and that
        // translation hits the same `core.h` SSE2 branch unless the
        // macro is set for the cImport too. Only for emscripten;
        // desktop / iOS / Android keep their native SIMD path.
        box2d_lib.root_module.addCMacro("BOX2D_DISABLE_SIMD", "1");
        mod.addCMacro("BOX2D_DISABLE_SIMD", "1");
    }

    // labelle-core: injected by the assembler at build time via addImport.
    // When building standalone (tests, remote fetch), use the lazy dependency.
    if (b.lazyDependency("labelle_core", .{ .target = target, .optimize = optimize })) |core_dep| {
        mod.addImport("labelle-core", core_dep.module("labelle-core"));
    }

    // Test step — compiles and analyses src/root.zig standalone.
    const tests = b.addTest(.{ .root_module = mod });
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&b.addRunArtifact(tests).step);
}
