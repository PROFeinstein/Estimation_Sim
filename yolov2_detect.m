function [Img,x,y] = yolov2_detect(in)
J = imresize(in, [224,224]);
%UNTITLED2 Summary of this function goes here
persistent yolov2Obj;
    if isempty(yolov2Obj)
     yolov2Obj = coder.loadDeepLearningNetwork('defet.mat');
    end
 
 [bboxes, scores, labels] = yolov2Obj.detect(J);
 Img = insertObjectAnnotation(J, 'rectangle', bboxes, labels);
 x =  (((bboxes(1,3)/2) + bboxes(1,1)) * 640)/224;
 y =  (((bboxes(1,4)/2) + bboxes(1,2))* 360)/224;
 %xy = (scores + 0)*1;
 %y = double(xy);
 
end
 
 
 
 

