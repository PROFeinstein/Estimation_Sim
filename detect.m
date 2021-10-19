
imageSize = [224 224 3];
numClasses =1;


anchorBoxes = [19    18;
    49    48;
   123   107;
    32    29;
    63    43;
    88    89;
    74    62;
    25    24;
    56    58;
    40    38;
    43    32

]
network = resnet50();
%analyzeNetwork(network)
featureLayer = 'activation_49_relu';
lgraph = yolov2Layers(imageSize,numClasses,anchorBoxes,network,featureLayer);
trainingData = objectDetectorTrainingData(gTruth)
lgraph.Layers
options = trainingOptions('sgdm',...
          'InitialLearnRate',0.001,...
          'Verbose',true,...
          'MiniBatchSize',32,...
          'MaxEpochs',500,...
          'Shuffle','every-epoch',...
          'VerboseFrequency',30);
      [defet,info] = trainYOLOv2ObjectDetector(trainingData,lgraph,options);