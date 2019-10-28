module IntersectionOverUnionUnitBall

using GeometryTypes: HyperRectangle, Vec
using Roots: find_zero, Bisection
include("PlotsGeometryTypes.jl")

measure(bb::HyperRectangle) = prod(bb.widths)
center(bb::HyperRectangle) = bb.origin .+ bb.widths./2

function iou(bb1::HyperRectangle, bb2::HyperRectangle)
  bb_ = bb1 ∩ bb2 # GeometryTypes intersection is imo broken for the empty case
  bb_i = HyperRectangle(bb_.origin, max.(0, bb_.widths))
  measure_i = measure(bb_i)
  measure_u = measure(bb1) + measure(bb2) - measure_i
  return measure_i / measure_u
end

"""
https://arxiv.org/abs/1902.09630
"""
function giou(bb1::HyperRectangle, bb2::HyperRectangle)
  bb_ = bb1 ∩ bb2 # GeometryTypes intersection is imo broken for the empty case
  bb_i = HyperRectangle(bb_.origin, max.(0, bb_.widths))
  measure_i = measure(bb_i)
  measure_u = measure(bb1) + measure(bb2) - measure_i

  smallest_enclosing = bb1 ∪ bb2
  measure_C = measure(smallest_enclosing)
  return (measure_i / measure_u) - (1 - measure_u / measure_C)
end

"""
linear interpolation in parameter space
"""
function interpolate(bb1::HyperRectangle, bb2::HyperRectangle, α)
  origin = (1-α) * bb1.origin + α * bb2.origin
  widths = (1-α) * bb1.widths + α * bb2.widths
  return HyperRectangle(origin, widths)
end

bb_reference = HyperRectangle(Vec(-0.5, -0.5), Vec(1.0, 1.0))

function random_bb()
  possible_centers = HyperRectangle(Vec(-2.5, -2.5), Vec(5.0, 5.0))
  sampled_center = rand(Vec{2}) .* possible_centers.widths + possible_centers.origin

  widths_max = abs.(5 * (sampled_center - center(bb_reference)))
  sampled_widths = rand(Vec{2}) .* widths_max
  @assert all(sampled_widths .> 0)
  return HyperRectangle(sampled_center .- sampled_widths ./2, sampled_widths)
end

function hunt_iou(bb_ref, bb1, bb2, iou_goal, iou=iou)
  @assert iou_goal < 1
  family(α) = interpolate(bb1, bb2, α)
  f(α) = iou(bb_ref, family(α)) - iou_goal
  @assert f(1) < 0
  @assert f(0) > 0

  α_star = find_zero(f, (0,1), Bisection())
  return family(α_star)
end

function random_bb_at_iou(bb_ref, iou_goal)
  while true
    rand_bb = random_bb()
    if iou(bb_ref, rand_bb) >= iou_goal
      continue
    end
    return hunt_iou(bb_ref, bb_ref, rand_bb, iou_goal)
  end
end

ball_samples(n) = [random_bb_at_iou(bb_reference, 0.5) for _=1:n]

using Plots: Plots, Shape
using GeometryTypes: Point

function start_radius(::typeof(iou), bb_ref, widths_o)
  over_sqrt_2 = 1.5
  # find radius which ensures iou=0
  r = over_sqrt_2 * (maximum(widths_o) + maximum(bb_ref.widths))
end

function start_radius(::typeof(giou), bb_ref, widths_o)
  r = 1e3
end

"""
returns path (path[1] ≠ path[end]) of "unit circle"
"""
function iou_ball(iou_radius, bb_ref, widths, n, iou=iou)
  widths_o = Vec(widths...)
  r = start_radius(iou, bb_ref, widths_o)
  centers = (r * Vec(cosd(θ), sind(θ)) for θ in range(0, 360, length=n+1)[1:end-1])
  bb_others = (HyperRectangle(center  .- widths_o/2, widths_o) for center in centers)
  bb_1 = HyperRectangle(center(bb_ref) .- widths_o/2, widths_o)
  [hunt_iou(bb_ref, bb_1, bb_other, iou_radius, iou) for bb_other in bb_others]
end

end # module
