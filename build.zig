const std = @import("std");
const builtin = @import("builtin");

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

    // Android cross-compilation: Zig ships no libc headers for the
    // Android target, so box2d's C sources cannot find <math.h> etc.
    // (35x `'math.h' file not found` in box2d/math_functions.h). Those
    // headers live in the Android NDK sysroot. Plumb the NDK sysroot
    // include paths into both the box2d C library artifact (where the C
    // sources compile) and the plugin module (which @cImports the box2d
    // headers) — mirrors the iOS `ios_sdk_path` and emscripten blocks
    // above, and labelle-bgfx's own Android sysroot wiring. Gated on the
    // Android ABI so desktop / iOS / wasm builds are untouched.
    if (target.result.abi == .android or target.result.abi == .androideabi) {
        applyAndroidNdkSysroot(b, box2d_lib.root_module, target);
        applyAndroidNdkSysroot(b, mod, target);
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

/// Add the Android NDK sysroot include/library paths + API-level define
/// to a C-compiling module so its C sources and @cImport translation
/// find Bionic's <math.h>/<stdlib.h>/etc. Mirrors labelle-bgfx's
/// `applyNdkSysroot`. Panics with an actionable message if the NDK
/// can't be located (only called on the Android path).
fn applyAndroidNdkSysroot(b: *std.Build, mod: *std.Build.Module, target: std.Build.ResolvedTarget) void {
    const sysroot = androidNdkSysroot(b) orelse
        @panic("Could not find Android NDK. Set ANDROID_NDK_HOME or ANDROID_HOME.");
    const triple: []const u8 = switch (target.result.cpu.arch) {
        .aarch64 => "aarch64-linux-android",
        .x86_64 => "x86_64-linux-android",
        .arm, .thumb => "arm-linux-androideabi",
        .x86 => "i686-linux-android",
        else => @panic("unsupported Android arch for box2d"),
    };
    // Match the toolkit's default Android min_sdk (28).
    const api = "28";
    mod.addSystemIncludePath(.{ .cwd_relative = b.pathJoin(&.{ sysroot, "usr/include" }) });
    mod.addSystemIncludePath(.{ .cwd_relative = b.pathJoin(&.{ sysroot, "usr/include", triple }) });
    mod.addLibraryPath(.{ .cwd_relative = b.pathJoin(&.{ sysroot, "usr/lib", triple, api }) });
    mod.addCMacro("__ANDROID_API__", api);
    // Android .so consumers need PIC in every archived .o.
    mod.pic = true;
}

/// Locate the Android NDK sysroot, mirroring labelle-bgfx's resolver:
/// `ANDROID_NDK_HOME` first, then `ANDROID_HOME/ndk/<latest>`. Uses the
/// Zig 0.16 `std.Io.Dir` APIs (getEnvVarOwned / std.fs.cwd() removed).
/// Returns null if neither resolves to an existing sysroot.
fn androidNdkSysroot(b: *std.Build) ?[]const u8 {
    const io = b.graph.io;
    const host_tag = switch (builtin.os.tag) {
        .linux => "linux-x86_64",
        .macos => "darwin-x86_64",
        .windows => "windows-x86_64",
        else => "linux-x86_64",
    };
    // 1. ANDROID_NDK_HOME
    if (b.graph.environ_map.get("ANDROID_NDK_HOME")) |ndk_home| {
        const sysroot = b.pathJoin(&.{ ndk_home, "toolchains", "llvm", "prebuilt", host_tag, "sysroot" });
        if (std.Io.Dir.cwd().access(io, sysroot, .{})) |_| return sysroot else |_| {}
    }
    // 2. ANDROID_HOME/ndk/<latest>
    if (b.graph.environ_map.get("ANDROID_HOME")) |home| {
        const ndk_dir = b.pathJoin(&.{ home, "ndk" });
        var dir = std.Io.Dir.cwd().openDir(io, ndk_dir, .{ .iterate = true }) catch return null;
        defer dir.close(io);
        var latest: ?[]const u8 = null;
        var iter = dir.iterate();
        while (iter.next(io) catch null) |entry| {
            if (entry.kind == .directory) {
                if (latest) |prev| {
                    if (std.mem.order(u8, entry.name, prev) == .gt) {
                        b.allocator.free(prev);
                        latest = b.allocator.dupe(u8, entry.name) catch null;
                    }
                } else {
                    latest = b.allocator.dupe(u8, entry.name) catch null;
                }
            }
        }
        if (latest) |version| {
            defer b.allocator.free(version);
            const sysroot = b.pathJoin(&.{ ndk_dir, version, "toolchains", "llvm", "prebuilt", host_tag, "sysroot" });
            if (std.Io.Dir.cwd().access(io, sysroot, .{})) |_| return sysroot else |_| {}
        }
    }
    return null;
}
