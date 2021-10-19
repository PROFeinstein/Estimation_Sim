Dataset = objectDetectorTrainingData(gTruth);
%summary(Dataset)
%allBoxes = vertcat(Dataset.weed{:}); %Dataset.sedge{:}, Dataset.chenopodium{:}, Dataset.circium{:}%
%aspectRatio = allBoxes(:,1) ./ allBoxes(:,4);
%area = prod(allBoxes(:,3:4),2);
%allBoxes1 = Dataset.weed{1:197};
blds = boxLabelDatastore(Dataset(:,2));

%figure
%scatter(area,aspectRatio)
%xlabel("Box Area")
%ylabel("Aspect Ratio (width/height)");
%title("Box area vs. Aspect ratio")
numAnchors = 11;

[anchorBoxes,meanIoU] = estimateAnchorBoxes(blds, numAnchors);
anchorBoxes
meanIoU
maxNumAnchors = 15;
meanIoU = zeros([maxNumAnchors,1]);
anchorBoxes = cell(maxNumAnchors, 1);
for k = 1:maxNumAnchors
    % Estimate anchors and mean IoU.
    [anchorBoxes{k},meanIoU(k)] = estimateAnchorBoxes(trainingData,k);    
end

figure
plot(1:maxNumAnchors,meanIoU,'-o')
ylabel("Mean IoU")
xlabel("Number of Anchors")
title("Number of Anchors vs. Mean IoU")
% Cluster using K-Medoids.
%[clusterAssignments, anchorBoxes, sumd] = kmedoids(allBoxes(:,3:4),numAnchors,'Distance',@iouDistanceMetric);

% Display estimated anchor boxes. The box format is the [width height].
%anchorBoxes
%figure
%gscatter(area,aspectRatio,clusterAssignments);
%title("K-Mediods with "+numAnchors+" clusters")
%xlabel("Box Area")
%ylabel("Aspect Ratio (width/height)");
%grid
% Count number of boxes per cluster. Exclude the cluster center while
% counting.
%counts = accumarray(clusterAssignments, ones(length(clusterAssignments),1),[],@(x)sum(x)-1);

% Compute mean IoU.
%meanIoU = mean(1 - sumd./(counts))

%maxNumAnchors = 7;
%for k = 1:maxNumAnchors
    
    % Estimate anchors using clustering.
    %[clusterAssignments, anchorBoxes, sumd] = kmedoids(allBoxes(:,3:4),k,'Distance',@iouDistanceMetric);
    
    % Compute mean IoU.
    %counts = accumarray(clusterAssignments, ones(length(clusterAssignments),1),[],@(x)sum(x)-1);
    %meanIoU(k) = mean(1 - sumd./(counts));
%end

%figure
%plot(1:maxNumAnchors, meanIoU,'-o')
%ylabel("Mean IoU")
%xlabel("Number of Anchors")
%title("Number of Anchors vs. Mean IoU")