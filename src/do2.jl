using Plots
using Printf
using IntersectionOverUnionUnitBall: iou_ball, bb_reference, center, iou
using GeometryTypes: Point, HyperRectangle, Vec
using PolygonMinkowskiSum: SimplePolygon, minkowski_sum

# maybe broadcast (.+) is more appropriate
function Base.:+(point::Point, poly::SimplePolygon)
    SimplePolygon(Ref(point) .+ poly.points)
end

function make_once()
    once_happened = nothing

    function once(fig_ij, label)
        if once_happened === nothing
            once_happened = fig_ij
        end
        if once_happened == fig_ij label else () end
    end
end

subfigure_center_spacing = 2.5
iou_radius = 0.4
widths_range = (0.3, 1.7)

widths_range = range(0.5, 1.5, length=8)

p = Plots.plot(; title=(@sprintf "iou= %0.2f" iou_radius), aspect_ratio=1, legend=false)

once = make_once()

for (fig_i, w) = enumerate(widths_range), (fig_j, h) = enumerate(widths_range)
    fig_ij = (fig_i, fig_j)
    other_widths = Vec(w, h)
    fig_center = Point((subfigure_center_spacing .* fig_ij)...)


    sweepme = HyperRectangle(Vec(0, 0) - other_widths/2, other_widths)
    max_iou = iou(sweepme, bb_reference)
    @show max_iou
    if max_iou <= iou_radius
        # position of `sweepme` has enough iou with bb_reference
        continue
    end

    center_path = map(x->Point(center(x)...), iou_ball(iou_radius, bb_reference, other_widths, 200))
    outer = minkowski_sum(SimplePolygon(sweepme), SimplePolygon(center_path))

    Plots.plot!(p, fig_center + SimplePolygon(center_path);
        fillalpha=0, linecolor=:black, linestyle=:solid, once(fig_ij, (label="center",))...)
    Plots.plot!(p, fig_center + outer;
        fillalpha=0, linecolor=:red, linestyle=:dot, once(fig_ij, (label="outer",))...)
    Plots.plot!(p, fig_center + SimplePolygon(bb_reference);
        fillalpha=0.2, color=:blue, once(fig_ij, (label="reference",))...)

    example_p = center_path[1]
    example = HyperRectangle(sweepme.origin + example_p, sweepme.widths)
    Plots.plot!(p, fig_center + SimplePolygon(example);
        fillalpha=0.2, color=:green, once(fig_ij, (label="example",))...)
    #=
    Plots.scatter!(p, [fig_center + example_p];
        color=:green, once(fig_ij, (label="example",))...)
    =#
end
display(p)
