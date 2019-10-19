using IntersectionOverUnionUnitBall: ball_samples, iou, bb_reference
using NearestNeighbors
using SparseArrays
using GeometryTypes: Point, Vec
using LinearAlgebra

using GeometryTypes: HyperRectangle
using Plots: Shape
using Plots

function Shape(r::HyperRectangle{2})
  (w, h) = r.widths ./ 2
  (x, y) = r.origin
  Shape(x .+ [-w,w,w,-w], y .+ [-h,-h,h,h])
end

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



for i in walk(4000)[1:4:end]
  p = Plots.plot(Shape(bb_reference);
    aspect_ratio=1, xlims=(-2,2), ylims=(-2,2))
  Plots.plot!(p, Shape(samples[i]); fillalpha=0.2, fillcolor=:orange)
  display(p)
end

#=
dist_mat_I = Int64[]
dist_mat_J = Int64[]
dist_mat_V = Float64[]
coo = Dict{Tuple{Int64, Int64}, Float64}()

function update_distance(i, j, v)
  push!(dist_mat_I, i)
  push!(dist_mat_J, j)
  push!(dist_mat_V, v)
  # symmetric
  #push!(dist_mat_I, j)
  #push!(dist_mat_J, i)
  #push!(dist_mat_V, v)
end

for idx1 = 1:N
  for idx2 in idxs_nn[idx1]
    v = iou(samples[idx1], samples[idx2])
    update_distance(idx1, idx2, 1-v)
    coo[(idx1, idx2)] = 1-v
  end
end

m_dist = sparse(dist_mat_I, dist_mat_J, dist_mat_V)

include("tsp.jl")

function distance_f(from_i, to_i)
  idx = (from_i + 1, to_i + 1)
  @show idx
  if haskey(coo, idx)
    Int(round(100000 * coo[idx]))
  else
    1000000000
  end
end

prob_sol = setup_solve_tsp(distance_f, N)
tour = extract_tour(prob_sol)
=#