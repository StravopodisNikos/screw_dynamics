function [DCI,Wd,tpi] = ExhaustiveConfigSpaceEvaluation_6DoF_MMD(config,xi_ai,xi_pi,gsli0,gst0,Mib0)
% for each (θ1,θ2,θ3), metamorphosis for all (θpi i=1:6) and calculates Dynamic Performance Measures

cnt = 0; % anatomy counter
for tp1=-1.5708:0.7854:1.5708
    for tp2=-1.5708:0.7854:1.5708
        for tp3=-1.5708:0.7854:1.5708
            for tp4=-1.5708:0.7854:1.5708
                for tp5=-1.5708:0.7854:1.5708
                    for tp6=-1.5708:0.7854:1.5708
                        cnt=cnt+1;
                        % 1.Build anatomy vector
                        tpi(:,cnt) = [tp1 tp2 tp3 tp4 tp5 tp6]';
                        % 2.Calculate values affected by tpi changes;
                        [exp_pi,Pi] = CalculatePseudoExponentials(xi_pi,tpi(:,cnt));
                        % 3.Compute Jacobian
                        [Js,Jb,Error] = CalculateMetamorphicJacobians_6DoF(config,xi_ai,tpi(:,cnt),xi_pi,Pi,gst0);
                        % 4.Compute CoM Jacobian 
                        [Jbsli_ref,Jssli_ref,Jbsli_POE_ref,gsli_ref] = CalculateCoMBodyJacobians_6DoF(config,xi_ai,Pi,gsli0);
                        % 5.Compute Mass matrix
                        [Mb] = CalculateOnlyBodyMassMatrix(Jbsli_ref,Mib0);
                        % 6.Calculate Dynamic Performance Measures values
                        [DCI(cnt)] = CalculateDynamicConditioningIndex(Mb,6);
                        [Wd(cnt)] = calculateDynamicManipulabilityIndex(Js,Mb);
                    end
                end
            end
        end
    end
end

end