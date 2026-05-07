const std = @import("std");
const matrices = @import("matrices.zig");

pub fn With(N: type) type {
    return struct {
        pub const Mat3 = matrices.Mat3(N);
        pub const Mat4 = matrices.Mat4(N);
        pub const Quat = @import("quaternion.zig").Quat(N);
        pub const Vec2 = @Vector(2, N);
        pub const Vec3 = @Vector(3, N);
        pub const Vec4 = @Vector(4, N);

        // --- VECTOR CONSTANTS --- \\
        pub fn zero(T: type) T {
            return @splat(0.0);
        }

        // --- VECTOR COMPARISON --- \\
        pub fn eql(l: anytype, r: @TypeOf(l)) bool {
            return @reduce(.And, l == r);
        }

        // --- VECTOR MATH --- \\

        /// Returns the angle in radians between `l` and `r`. Assumes both
        /// vectors are normalized.
        pub fn angle(l: anytype, r: @TypeOf(l)) N {
            return std.math.acos(dot(l, r));
        }

        /// Clamps the magnitude of `v` to be no more than `max_length`.
        pub fn clampMagnitude(v: anytype, max_length: N) @TypeOf(v) {
            const sqr_len = sqrMagnitude(v);
            if (sqr_len <= max_length * max_length) return v;

            const len: @TypeOf(v) = @splat(@sqrt(sqr_len));
            const max: @TypeOf(v) = @splat(max_length);
            return v / len * max;
        }

        /// Returns the cross product of `l` and `r`.
        pub fn cross(l: Vec3, r: Vec3) Vec3 {
            const lx, const ly, const lz = l;
            const rx, const ry, const rz = r;
            const cx = ly * rz - lz * ry;
            const cy = lz * rx - lx * rz;
            const cz = lx * ry - ly * rx;
            return .{ cx, cy, cz };
        }

        /// Returns the distance between `l` and `r`.
        pub fn distance(l: anytype, r: @TypeOf(l)) N {
            return magnitude(l - r);
        }

        /// Returns the dot product of `l` and `r`.
        pub fn dot(l: anytype, r: @TypeOf(l)) N {
            return @reduce(.Add, l * r);
        }

        /// Extends `v` to be a vector with one more element.
        pub fn extend(n: comptime_int, v: @Vector(n, N), scalar: N) @Vector(n + 1, N) {
            var w: @Vector(n + 1, N) = undefined;
            inline for (0..n) |i| w[i] = v[i];
            w[n] = scalar;
            return w;
        }

        /// Returns the linear interpolation of `start` and `end` by `t`. `t`
        /// may be greater than one or less than zero, and this function will
        /// overshoot or undershoot, respectively.
        pub fn lerp(start: anytype, end: @TypeOf(start), t: N) @TypeOf(start) {
            const m = end - start;
            const t_v: @TypeOf(start) = @splat(t);
            return m * t_v + start;
        }

        /// Returns the length of `v`.
        pub fn magnitude(v: anytype) N {
            return @sqrt(sqrMagnitude(v));
        }

        /// Returns `current` moved closer to `target` by at most `max_delta`
        /// units. This will not overshoot.
        pub fn moveTowards(current: anytype, target: @TypeOf(current), max_delta: N) @TypeOf(current) {
            const delta = target - current;
            const distance_sqr = sqrMagnitude(delta);
            if (distance_sqr <= max_delta * max_delta) return target;

            const direction = delta / @as(@TypeOf(current), @splat(@sqrt(distance_sqr)));
            const d: @TypeOf(current) = @splat(max_delta);
            return current + direction * d;
        }

        /// Normalizes `v`. If the length of `v` is zero, this returns the zero
        /// vector.
        pub fn normalize(v: anytype) @TypeOf(v) {
            const m = magnitude(v);
            if (m == 0.0) return v;

            const len: @TypeOf(v) = @splat(m);
            return v / len;
        }

        /// Returns the orthonormal basis constructed from `v`.
        pub fn orthonormalBasis(v: Vec3) struct {
            tangent: Vec3,
            bitangent: Vec3,
        } {
            const s = if (std.math.signbit(v[2])) @as(f32, -1.0) else @as(f32, 1.0);
            const a = -1.0 / (s + v[2]);
            const b = v[0] * v[1] * a;
            return .{
                .tangent = Vec3{ 1.0 + s * v[0] * v[0] * a, s * b, -s * v[0] },
                .bitangent = Vec3{ b, s + v[1] * v[1] * a, -v[1] },
            };
        }

        /// Returns the 2D vector perpendicular to `v`. This is equivalent to
        /// rotating `v` 90 degrees counter-clockwise.
        pub fn perpendicular(v: Vec2) Vec2 {
            return .{ -v[1], v[0] };
        }

        /// Projects `v` onto `normal`.
        pub fn project(v: anytype, normal: @TypeOf(v)) @TypeOf(v) {
            const d: @TypeOf(v) = @splat(dot(v, normal));
            return normal * d;
        }

        /// Reflects `direction` across `normal`.
        pub fn reflect(direction: anytype, normal: @TypeOf(direction)) @TypeOf(direction) {
            const d: @TypeOf(direction) = @splat(dot(direction, normal) * 2.0);
            return direction - d * normal;
        }

        /// Rotates `v` by `theta` radians counter-clockwise.
        pub fn rotate(v: Vec2, theta: N) Vec2 {
            const x, const y = v;
            const cos = @cos(theta);
            const sin = @sin(theta);
            return .{ x * cos - y * sin, x * sin + y * cos };
        }

        /// Roates `current` towards `target` by at most `max_delta` radians.
        /// This will not overshoot.
        pub fn rotateToward(current: Vec3, target: Vec3, max_delta: N) Vec3 {
            const current_len: Vec3 = @splat(magnitude(current));
            const c = current / current_len;
            const t = normalize(target);
            const d = dot(c, t);
            const a = std.math.acos(d);
            if (a == 0.0) return target;

            const r = @min(a, max_delta);
            var axis = cross(c, t);
            const axis_len = magnitude(axis);

            var p: Vec3 = undefined;
            if (axis_len == 0.0) {
                // Vectors are antiparallel
                p = if (@abs(c[0]) < 0.9) .{ 1, 0, 0 } else .{ 0, 1, 0 };
                axis = normalize(cross(c, p));
            }

            const c_cos_r = c * @as(Vec3, @splat(@cos(r)));
            const axis_cross_c_sin_r = cross(axis, c) * @as(Vec3, @splat(@sin(r)));
            const axis_dot_c_cos_r: Vec3 = @splat(dot(axis, c) * (1.0 - @cos(r)));
            return (c_cos_r + axis_cross_c_sin_r + axis_dot_c_cos_r) * current_len;
        }

        /// Uniformly scales `v` by `scalar`.
        pub fn scale(v: anytype, scalar: N) @TypeOf(v) {
            const s: @TypeOf(v) = @splat(scalar);
            return v * s;
        }

        /// Returns the signed angle between `l` and `r`.
        pub fn signedAngle(l: Vec2, r: Vec2) N {
            return std.math.atan2(l[0] * r[1] - l[1] * r[0], l[0] * r[0] + l[1] * r[1]);
        }

        /// Spherically interpolates between `start` and `target` by `t`. `t`
        /// can be greater than 1 or less than 0 and this function will overshoot
        /// or undershoot, respectively.
        pub fn slerp(start: anytype, target: @TypeOf(start), t: N) @TypeOf(start) {
            const start_len = magnitude(start);
            const target_len = magnitude(target);
            if (start_len == 0.0 and target_len == 0.0) return @splat(0.0);
            if (start_len == 0.0) return target * @as(@TypeOf(start), @splat(t / target_len));
            if (target_len == 0.0) return start * @as(@TypeOf(start), @splat((1.0 - t) / start_len));

            const start_norm = start / @as(@TypeOf(start), @splat(start_len));
            const target_norm = target / @as(@TypeOf(target), @splat(target_len));
            const d = dot(start_norm, target_norm);
            if (d == 1.0) return target;

            const theta = std.math.acos(d);
            const sin_theta = @sin(theta);
            const sin_theta_t = @sin(theta * t);
            const sin_theta_1_minus_t = @sin(theta * (1.0 - t));

            const s0: @TypeOf(start) = @splat(sin_theta_1_minus_t / sin_theta);
            const s1: @TypeOf(start) = @splat(sin_theta_t / sin_theta);
            return (start_norm * s0 + target_norm * s1) * @as(@TypeOf(start), @splat(target_len));
        }

        /// Returns the length of `v` squared. This is faster than `magnitude`.
        pub fn sqrMagnitude(v: anytype) N {
            return dot(v, v);
        }
    };
}
