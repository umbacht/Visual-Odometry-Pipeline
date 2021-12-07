function [last20FramesIndices, numberOfTrackedLandmarks_all, ...
            allWorldCoordinates] = ...
                 plotting(S_curr, T_WC_curr, img_curr, ...
                    numberOfTrackedLandmarks_all, allWorldCoordinates, ...
                    last20FramesIndices)
                
    % update plot data
    currI = last20FramesIndices(end);
    allWorldCoordinates(:, currI) = T_WC_curr([1, 3], end);
    numberOfTrackedLandmarks_all(currI) = size(S_curr.X, 1);    
                
    % initialize plot and make full screen
    figure(2)
    set(gcf, 'Units','normalized','Position',[0 0 1 1]) 
    
    % upper left corner plot
    subplot(4,4,[1,2,5,6])
    imshow(img_curr)
    hold on
    plot(S_curr.C(:, 1), S_curr.C(:, 2), 'rx');
    plot(S_curr.P(:, 1), S_curr.P(:, 2), 'gx')
    hold off
    title('Current Image')
    
    % bottom left corner left plot
    subplot(4,4,[9,13])
    plot(-19:0, numberOfTrackedLandmarks_all(last20FramesIndices))
    title('# tracked landmarks over last 20 frames.')

    % bottom left corner right plot
    subplot(4,4,[10,14])
    % start at nineteen because first 18 are zeros for easy init
    plot(allWorldCoordinates(1, 19:currI), allWorldCoordinates(2, 19:currI))
    axis equal
    title('Full trajectory.')
    
    
    % right plot
    r1 = subplot(4,4,[3,4,7,8,11,12,15,16]);
    cla(r1);
    hold on
    % only plot 20 points if there are 20 (currI starts at 20 and we 
    % already have 2 points)
    if currI < 38
        plot(allWorldCoordinates(1, 19:currI), ...
        allWorldCoordinates(2, 19:currI), 'b')
    else
        plot(allWorldCoordinates(1, last20FramesIndices), ...
        allWorldCoordinates(2, last20FramesIndices), 'b')
    end
    
    plot(S_curr.X(:, 1), S_curr.X(:, 3), 'gx')
    hold off   
    axis equal
    title('Trajectory of last 20 frames and landmarks')
    
    set(gcf, 'GraphicsSmoothing', 'on')
    
    % update indices
    last20FramesIndices = last20FramesIndices + 1;
end