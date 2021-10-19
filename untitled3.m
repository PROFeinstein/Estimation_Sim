I = imread('a.jpg');
%J = imresize(I, [224,224]);
[img,x,y] = yolov2_detect(I);
imtool(img)
%hwJetson = jetson('10.209.101.73','mahmoud','Mubiksium9');