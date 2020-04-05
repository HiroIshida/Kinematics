using Test
include("transform.jl")

function test_quaternion()
    rot1 = Quaternion([0, 0, 1], pi/2)
    rot2 = Quaternion(0, 0, pi/2)
    @test norm(rot1.vec - rot2.vec) < 1e-10
end
    
function test_rotation()
  rot43 = Quaternion(0, 0, -pi/2)
  rot32= Quaternion(0, -pi/2, 0)
  rot21= Quaternion(0, 0, pi/2)
  rot41 = rot43*rot32*rot21
  rot41_direct = Quaternion(pi/2, 0, 0)
  @test norm(rot41_direct.vec - rot41.vec) < 1e-10
end

function test_transform()
  tf21 = Transform([1., 1., 0.], [0, 0, pi/4])
  tf32 = Transform([0., 0., 0.], [0, 0, pi/4])
  tf43 = Transform([1., 0., 0.], [0, 0, 0])
  tf41 = tf43 ∘ tf32 ∘ tf21
  tf41_direct = Transform([1., 2., 0], [0, 0, pi/2])
  @test norm(tf41.trans - [1., 2., 0]) < 1e-10
  @test norm(tf41.rot.vec - rpy2quaternion(0, 0, pi/2)) < 1e-10
end

test_quaternion()
test_rotation()
test_transform()
