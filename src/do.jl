using IntersectionOverUnionUnitBall: ball_samples, iou, bb_reference
using NearestNeighbors
using SparseArrays
using GeometryTypes: Point, Vec
using LinearAlgebra

using GeometryTypes: HyperRectangle
using Plots: Shape
using Plots

N = 30000
k_nn = 20
samples = ball_samples(N)
#Plots.scatter(map(bb->Point(bb.origin...), samples); aspect_ratio=1)
param(bb) = Vec(bb.origin..., bb.widths...)

points = map(param, samples)

nn = KDTree(points)

idxs_nn, _dists = knn(nn, points, k_nn)

function walk(n_walk)
  visits = zeros(Int64, N)
  momentum = [0.0, 0.0, 0.0, 0.0]

  function score(i_next, i)
    if i_next == i
      return -Inf
    end
    near_bias = iou(samples[i_next], samples[i])
    visit_ratio = (visits[i_next] - minimum(visits) + 1) / (maximum(visits) - minimum(visits) + 1)
    visit_bias = 1 - visit_ratio
    Δp = LinearAlgebra.normalize(param(samples[i_next]) - param(samples[i]))
    momentum_bias = dot(Δp, momentum)
    s = near_bias + visit_bias + 10*momentum_bias
    return s
  end

  i = 1
  path = []
  for k=1:n_walk
    scores = ((score(i_next, i), i_next) for i_next in idxs_nn[i])
    _, i_next = maximum(scores)
    @assert i_next != i
    Δp = LinearAlgebra.normalize(param(samples[i_next]) - param(samples[i]))
    momentum .= (0.9 * momentum + 0.1 * Δp)
    if rand() > 0.985
      momentum = rand(4,) .- 0.5
    end
    push!(path, i_next)
    i = i_next
  end

  return path
end


anim = walk(4000)[1:4:end]
for i in anim
  p = Plots.plot(Shape(bb_reference);
    aspect_ratio=1, xlims=(-2,2), ylims=(-2,2))
  Plots.plot!(p, Shape(samples[i]); fillalpha=0.2, fillcolor=:orange)
  #Plots.plot!(p, Shape(bb_reference ∩ samples[i]), alpha=0.3, fillcolor=:green)
  display(p)
end
