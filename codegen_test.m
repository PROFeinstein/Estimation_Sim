img = imread('wat.jpg');
%im = imresize(img, [224,224]);
%figure, imshow(im);
yolov2Obj = load('defer.mat');
[bboxes, scores, labels] = detect(yolov2Obj.defet, img, 'Threshold', 0.4);
out = insertObjectAnnotation(img, 'rectangle', bboxes, cellstr(labels));
x = bboxes(1,1) + (bboxes(1,3)/2)
y = bboxes(1,2) + (bboxes(1,4)/2)


figure, imtool(out);