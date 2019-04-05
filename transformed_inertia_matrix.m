function [tf_inertia] = transformed_inertia_matrix(inertia,g)
% Transforms Inertia matrix of link i
% based on eq.4.28 p.176 Murray

% g must be the tf from intial frame to the end frame f.e.: b->c
%  Mc = Ad(inv(g_cb))'* Mb * Ad(inv(g_cb))

tf_inertia = ad(inv(g))'*inertia*ad(inv(g));

end