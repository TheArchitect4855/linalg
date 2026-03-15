# LinAlg
An OpenGL-compatible linear algebra library for the Zig programming language,
intended for use in game development and graphics programming.

⚠️ DISCLAIMER ⚠️

This library is in **very early** development. It has not been used in any
projects, and _very likely_ has bugs that you will encounter. I am not a
mathematician, (in fact I'm quite bad at math) so I will make no guarantees
about the accuracy or correctness of this library.

If you decide to use this library and you find a bug or find a confusing feature,
please submit an issue or a pull request. That's the only way this library can
improve!

## Features

1. Vectors, using Zig's `@Vector` builtin for SIMD goodness.
2. Quaternions, so you don't get gimbal locked
3. 4x4 matrices, because any other shape is inferior

## Background

I created this library because I think Zig is a great language for game/engine
development. I commonly use OpenGL for my projects, which is where the OpenGL
specialization comes from.

### Goals

In order of importance:

#### 1. Correctness.
While I'm bad at math, this is very important and is a high priority.

#### 2. Simplicity.
The library should make sense and be intuitive.

#### 3. Speed.
Operations should be fast. This library should never be the bottleneck.

### Non-Goals

#### General Purpose
This library is _not_ meant to be a general purpose linear algebra library. It
is meant to be an OpenGL graphics and game development linear algebra library.

## Installation

Since this library is in early development, it makes the most sense to install
the latest version to your Zig project:

`zig fetch --save git+https://github.com/TheArchitect4855/linalg.git`

Then in your `build.zig`:
```zig
const linalg = b.dependency("linalg", .{
	.target = target,
	.optimize = optimize,
});

exe_mod.addImport("linalg", linalg.module("linalg"));
```

## Usage

```zig
const linalg = @import("linalg").With(f32); // Linear algebra with f32!

const projection_matrix = linalg.Mat4.orthographic(-0.5, 0.5, -0.5, 0.5, 0.0, 100.0); // .perspective is also available

const quaternion = linalg.Quat.euler(0, std.math.pi * 0.5, 0); // Rotate 90 degrees on the Y axis

const vector = linalg.Vec3{1, 2, 3};
const normalized = linalg.vec.normalize(3, vector); // The 3 is the vector dimension; most functions are generic over all vectors
```

Browse the doc comments for more in-depth documentation.

## Support

Found a bug? Have a feature request? Submit an issue or create a PR!

## Contributing

Contributions are open! If you'd like to tackle an open issue, please leave a
comment to indicate you will be working on it. To submit your code, create a
pull request.
