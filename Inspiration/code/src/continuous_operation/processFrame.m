function [S_curr, T_WC_curr] = processFrame(I_curr, I_prev, S_prev, params, T_WC_prev)
    % After Initialization this is used to process each frame during
    % continuous operation of th VO.
    % The state of S of a frame is given as suggested in the project 
    % statement as following struct: S = {P, X}
    
    % Inputs:   
    %           I_curr      :       image of current frame
    %           I_prev      :       image of previous frame
    %           S_prev      :       state of previous frame
    %           params      :       Paramteres
    %           T_WC_prev   :       estimated pose of previos frame
    % Outputs:   
    %           S_curr      :       state of current frame
    %           T_WC_curr   :       pose of current frame
    
    %% Step 1 estimate pose
    % associate keypoints of current frame to landmarks and use this to
    % estimate pose of current frame
    % add function here something like this: [S_curr.P, S_curr.X, T_WC_curr] = estimateCurrentPose()
    [S_curr, T_WC_curr] = estimatePose(I_curr, I_prev, S_prev, params, T_WC_prev);
    
    %% Step 2 triangulate new landmarks
    S_curr = triangulateNewLandmarks(I_curr, I_prev, S_curr, T_WC_curr, params);
    
    %% Step 3 delete unnecessary landmarks
    
    % Compute position of landmark in camera frame
    X_c = (inv(T_WC_curr) * [S_curr.X, ones(length(S_curr.P), 1)]')';
    Z_c = X_c(:, 3);
    mean_z = mean(Z_c);
    std_z = std(Z_c);
    
    % Delete landmarks behind vehicle
    S_curr.X = S_curr.X(Z_c > 1 & Z_c < (mean_z + params.num_std*std_z), 1:3);
    S_curr.P = S_curr.P(Z_c > 1 & Z_c < (mean_z + params.num_std*std_z), :);
    
end