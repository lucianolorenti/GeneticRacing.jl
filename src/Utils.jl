function cross2D(v,w)
	v[1] * w[2] - v[2] * w[1];
end
function is_point_in_rectangle(A,B,C, M)
 return   ( 0 < dot(M-A,B-A) < dot(B-A,B-A)) && (0<dot(C-B, M-B) <dot(C-B,C-B))
end
function rotation_matrix(ang::T) where T<:Number
    return [cosd(ang) -sind(ang);sind(ang) cosd(ang)]
end
import Base.angle
function angle(v::Vector)
	local ang = atan2(v[2],v[1]) - atan2(0,1) + pi/2
    if ang<0
        ang=ang+2*pi
    end
	return ang
end
function intersection(line1, line2)
    p = line1[1]
    r = line1[2]-line1[1]
   q = line2[1]
   s = line2[2]-line2[1]
   r_s = cross2D(r, s);
   q_p_r = cross2D(q-p, r);

  if (abs(r_s)<0.000001 && abs(q_p_r)<0.0000001)
     t1 = dot(q+(s-p), r) / dot(r, r);
     t0 = t1 - dot(s, r) / dot(r, r);
    if (t0 >= 0 && t0 <= 1 || t1 >= 0 && t1 <= 1)
      return (true, t0, t1)
   end
	return (false, 0, 0)
 end

	if ((r_s)<0.0000001 && !(q_p_r<0.0000001))
    	return (false,0 ,0)
	end
  	 t = cross2D(q-p, s) / cross2D(r, s);
 	 u = cross2D(q-p, r) / cross2D(r, s);
    if (!(abs(r_s)<0.0000001) && t >= 0 && t <= 1 && u >= 0 && u <= 1)
	    return (true, t,u)
	end
  return (false,0,0)
end
