function standingPlanner!(p_ref::Vector{Float64}, o_ref::Vector{Float64}, t::Float64)
	freq 		= 	1
	phase 		= 	t * 2 * pi * freq
	p_ref 		.= [(sin(phase)*0.04), (cos(phase)*0.04), (sin(phase)*0.025 + 0.32)]
	o_ref		.= [(sin(phase)*5*pi/180.0 + 0*pi/180), (cos(phase)*5*pi/180.0 + 0*pi/180), (sin(phase)*5*pi/180.0 + 0*pi/180)]
end
