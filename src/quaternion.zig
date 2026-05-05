const std = @import("std");

/// A quaternion.
pub fn Quat(N: type) type {
    const v = @import("vectors.zig").Vectors(N);
    return packed struct {
        const Self = @This();

        pub const identity = Self{ .w = 1, .x = 0, .y = 0, .z = 0 };
        pub const zero = Self{ .w = 0, .x = 0, .y = 0, .z = 0 };

        w: N,
        x: N,
        y: N,
        z: N,

        // --- CONSTRUCTORS ---

        /// Creates a quaternion by rotating `theta` radians around `axis`.
        pub fn angleAxis(theta: N, axis: @Vector(3, N)) Self {
            const half_angle = theta * 0.5;
            const s = @sin(half_angle);
            return Self{
                .w = @cos(half_angle),
                .x = axis[0] * s,
                .y = axis[1] * s,
                .z = axis[2] * s,
            };
        }

        /// Creates a quaternion from euler angles. Applied in the order ZYX.
        /// Angles are in radians.
        pub fn euler(x: N, y: N, z: N) Self {
            const half_x = x * 0.5;
            const half_y = y * 0.5;
            const half_z = z * 0.5;

            const cx = @cos(half_x);
            const sx = @sin(half_x);
            const cy = @cos(half_y);
            const sy = @sin(half_y);
            const cz = @cos(half_z);
            const sz = @sin(half_z);

            return Self{
                .w = cx * cy * cz + sx * sy * sz,
                .x = sx * cy * cz - cx * sy * sz,
                .y = cx * sy * cz + sx * cy * sz,
                .z = cx * cy * sz - sx * sy * cz,
            };
        }

        /// Creates a quaternion "looking" in the direction defined by `forward` and `up`.
        pub fn lookRotation(forward: @Vector(3, N), up: @Vector(3, N)) Self {
            const right = v.cross(up, forward);
            const new_up = v.cross(forward, right);

            // Construct the rotation matrix
            var m: [3][3]N = undefined;

            // First row is the right vector
            m[0][0] = right[0];
            m[0][1] = right[1];
            m[0][2] = right[2];

            // Second row is the up vector
            m[1][0] = new_up[0];
            m[1][1] = new_up[1];
            m[1][2] = new_up[2];

            // Third row is the forward vector
            m[2][0] = forward[0];
            m[2][1] = forward[1];
            m[2][2] = forward[2];

            // Convert the rotation matrix to a quaternion
            const trace = m[0][0] + m[1][1] + m[2][2];
            if (trace > 0) {
                const s = 0.5 / @sqrt(trace + 1.0);
                return Self{
                    .w = 0.25 / s,
                    .x = (m[2][1] - m[1][2]) * s,
                    .y = (m[0][2] - m[2][0]) * s,
                    .z = (m[1][0] - m[0][1]) * s,
                };
            } else if (m[0][0] > m[1][1] and m[0][0] > m[2][2]) {
                const s = 2.0 * @sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]);
                return Self{
                    .w = (m[2][1] - m[1][2]) / s,
                    .x = 0.25 * s,
                    .y = (m[0][1] + m[1][0]) / s,
                    .z = (m[0][2] + m[2][0]) / s,
                };
            } else if (m[1][1] > m[2][2]) {
                const s = 2.0 * @sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]);
                return Self{
                    .w = (m[0][2] - m[2][0]) / s,
                    .x = (m[0][1] + m[1][0]) / s,
                    .y = 0.25 * s,
                    .z = (m[1][2] + m[2][1]) / s,
                };
            } else {
                const s = 2.0 * @sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]);
                return Self{
                    .w = (m[1][0] - m[0][1]) / s,
                    .x = (m[0][2] + m[2][0]) / s,
                    .y = (m[1][2] + m[2][1]) / s,
                    .z = 0.25 * s,
                };
            }
        }

        // --- METHODS ---

        /// Returns the angle in radians between `self` and `other`.
        pub fn angle(self: Self, other: Self) N {
            return 2.0 * std.math.acos(@abs(self.dot(other)));
        }

        /// Returns the conjugate of this quaternion. For unit quaternions, this
        /// is the same as the inverse, but calculating this is faster.
        pub fn conjugate(self: Self) Self {
            return .{
                .w = self.w,
                .x = -self.x,
                .y = -self.y,
                .z = -self.z,
            };
        }

        /// Returns the dot product of `a` and `b`.
        pub fn dot(a: Self, b: Self) N {
            return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        }

        /// Returns `true` if `self` is exactly equal to `other`.
        pub fn eql(self: Self, other: Self) bool {
            return self.w == other.w and self.x == other.x and self.y == other.y and self.z == other.z;
        }

        /// Returns `true` if `self` is approximately equal to `other`.
        pub fn eqlApprox(self: Self, other: Self, tolerance: f32) bool {
            const w = std.math.approxEqAbs(N, self.w, other.w, tolerance);
            const x = std.math.approxEqAbs(N, self.x, other.x, tolerance);
            const y = std.math.approxEqAbs(N, self.y, other.y, tolerance);
            const z = std.math.approxEqAbs(N, self.z, other.z, tolerance);
            return w and x and y and z;
        }

        /// Format function for printing. One can use the `{d}` format specifier in the same was as printing a number.
        pub fn format(self: Self, comptime fmt: []const u8, options: std.fmt.FormatOptions, writer: anytype) !void {
            _ = options;
            try writer.print("[ {" ++ fmt ++ "}i {" ++ fmt ++ "} {" ++ fmt ++ "} {" ++ fmt ++ "} ]", .{ self.w, self.x, self.y, self.z });
        }

        /// Returns the imaginary part of this quaternion (i.e. the x, y, z
        /// components)
        pub fn imaginary(self: Self) @Vector(3, N) {
            return .{ self.x, self.y, self.z };
        }

        /// Returns the inverse of this quaternion.
        pub fn inverse(self: Self) Self {
            const len_sq = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
            const inv_len_sq = 1.0 / len_sq;
            return Self{
                .w = self.w * inv_len_sq,
                .x = -self.x * inv_len_sq,
                .y = -self.y * inv_len_sq,
                .z = -self.z * inv_len_sq,
            };
        }

        /// Interpolates between `a` and `b` by factor `t`.
        pub fn lerp(a: Self, b: Self, t: N) Self {
            return Self.lerpUnclamped(a, b, std.math.clamp(t, 0.0, 1.0));
        }

        /// Interpolates between `a` and `b` by factor `t`. Allows over- and under-shooting.
        pub fn lerpUnclamped(a: Self, b: Self, t: N) Self {
            var result = Self{
                .w = a.w * (1.0 - t) + b.w * t,
                .x = a.x * (1.0 - t) + b.x * t,
                .y = a.y * (1.0 - t) + b.y * t,
                .z = a.z * (1.0 - t) + b.z * t,
            };

            return result.normalized();
        }

        /// Multiplies `self` with `other`. This is equivalent to combining two rotations.
        pub fn mul(self: Self, other: Self) Self {
            return Self{
                .w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
                .x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
                .y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
                .z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            };
        }

        /// Multiplies this quaternion by `scalar`.
        pub fn mulScalar(self: Self, scalar: N) Self {
            return .{
                .w = self.w * scalar,
                .x = self.x * scalar,
                .y = self.y * scalar,
                .z = self.z * scalar,
            };
        }

        /// Converts `vector` to a pure quaternion and multiplies this
        /// quaternion with it.
        pub fn mulVector(self: Self, vector: @Vector(3, N)) Self {
            const vx, const vy, const vz = vector;
            return .{
                .w = -self.x * vx - self.y * vy - self.z * vz,
                .x = self.w * vx + self.y * vz - self.z * vy,
                .y = self.w * vy - self.x * vz + self.z * vx,
                .z = self.w * vz + self.x * vy - self.y * vx,
            };
        }

        /// Normalizes this quaternion.
        pub fn normalized(self: Self) Self {
            const len = @sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
            return Self{
                .w = self.w / len,
                .x = self.x / len,
                .y = self.y / len,
                .z = self.z / len,
            };
        }

        /// Rotates this quaternion around the axis defined by `axis`. The
        /// amount of rotation is determined by the magnitude of `axis`.
        pub fn rotateByAxis(self: Self, axis: @Vector(3, N)) Self {
            const dq = self.mulVector(axis).mulScalar(0.5);
            return .{
                .w = self.w + dq.w,
                .x = self.x + dq.x,
                .y = self.y + dq.y,
                .z = self.z + dq.z,
            };
        }

        /// Rotates `current` towards `target`, by at most `max_angle_delta` radians. Will not overshoot.
        pub fn rotateTowards(current: Self, target: Self, max_angle_delta: f32) Self {
            const angle_val = current.angle(target);
            if (angle_val == 0.0) return target;
            if (angle_val <= max_angle_delta) return target;

            // Otherwise, we need to do a partial rotation using slerp
            const t = max_angle_delta / angle_val;
            return Self.slerpUnclamped(current, target, t);
        }

        /// Spherically interpolates between `a` and `b` by factor `t`.
        pub fn slerp(a: Self, b: Self, t: N) Self {
            return a.slerpUnclamped(b, std.math.clamp(t, 0.0, 1.0));
        }

        /// Spherically interpolates between `a` and `b` by factor `t`. Allows over- and under-shooting.
        pub fn slerpUnclamped(a: Self, b: Self, t: N) Self {
            // Spherical Linear Interpolation for quaternions
            var end = b;

            // Calculate cosine of angle between quaternions
            var cos_theta = Self.dot(a, end);

            // If the dot product is negative, negate one of the quaternions
            // to take the shorter path
            if (cos_theta < 0.0) {
                end = Self{
                    .w = -end.w,
                    .x = -end.x,
                    .y = -end.y,
                    .z = -end.z,
                };
                cos_theta = -cos_theta;
            }

            // Set default factors in case of close quaternions
            var factor1 = 1.0 - t;
            var factor2 = t;

            // If the quaternions are not close, use spherical interpolation
            if (cos_theta < 1.0) {
                const theta = std.math.acos(cos_theta);
                const sin_theta = @sin(theta);

                factor1 = @sin((1.0 - t) * theta) / sin_theta;
                factor2 = @sin(t * theta) / sin_theta;
            }

            // Calculate the interpolated quaternion
            return Self{
                .w = a.w * factor1 + end.w * factor2,
                .x = a.x * factor1 + end.x * factor2,
                .y = a.y * factor1 + end.y * factor2,
                .z = a.z * factor1 + end.z * factor2,
            };
        }
    };
}
