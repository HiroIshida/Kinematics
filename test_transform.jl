using Test
include("transform.jl")

function test_rotation()
  rot43 = Quaternion(0, 0, -pi/2)
  rot32= Quaternion(0, -pi/2, 0)
  rot21= Quaternion(0, 0, pi/2)
  rot41 = rot43*rot32*rot21
  rot41_direct = Quaternion(pi/2, 0, 0)
  @test norm(rot41_direct.vec - rot41.vec) < 1e-10
end

test_transform()
