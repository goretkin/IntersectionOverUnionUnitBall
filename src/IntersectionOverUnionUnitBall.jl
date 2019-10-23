module IntersectionOverUnionUnitBall

using GeometryTypes: HyperRectangle, Vec
using Roots: find_zero, Bisection

measure(bb::HyperRectangle) = prod(bb.widths)
center(bb::HyperRectangle) = bb.origin .+ bb.widths./2

function iou(bb1::HyperRectangle, bb2::HyperRectangle)
  measure_i = measure(bb1 ∩ bb2)
  measure_u = measure(bb1) + measure(bb2) - measure_i
  return measure_i / measure_u
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

function hunt_iou(bb_ref, bb1, bb2, iou_goal)
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

end # module
