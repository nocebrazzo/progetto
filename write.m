im1 = double(imread('0.png'))/255;
im2=double(imread('357.png'))/255;
im1size = size(im1);
im2size = size(im2);
width1 = im1size(2);
width2 = im2size(2);
imdiff = abs(im1(:,1:width1) - im2(:,1:width2));
imwrite(imdiff, 'result.png');
imshowpair(im1,imdiff,'montage');