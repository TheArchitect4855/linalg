pub fn LinAlg(N: type) type {
    return struct {
        pub const Mat4 = @import("matrices.zig").Mat4(N);
        pub const Quat = @import("quaternion.zig").Quat(N);
        pub const Vec2 = @Vector(2, N);
        pub const Vec3 = @Vector(3, N);
        pub const Vec4 = @Vector(4, N);
        pub const vec = @import("vectors.zig").Vectors(N);
    };
}
