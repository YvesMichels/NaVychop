im = imread('cameraman.tif');
im_f = uint16(im);

im_f = imfilter(im_f, [1,1,1]);
im_f = imfilter(im_f, [1;1;1]);
im_f = uint16(im_f)*14;
im_f = im_f/(16*8);
im_f = imfilter(im_f, [1,1,1]);
im_f = imfilter(im_f, [1;1;1]);
im_f = uint16(im_f)*14;
im_f = im_f/(16*8);
im_f = imfilter(im_f, [1,1,1]);
im_f = imfilter(im_f, [1;1;1]);
im_f = uint16(im_f)*14;
im_f = im_f/(16*8);

ker_15 = [2, 4, 8, 13, 21, 28, 34, 36, 34, 28, 21, 13, 4, 2];
im_g = uint16(im);
im_g = imfilter(im_g, ker_15);
im_g = im_g/256;
im_g = imfilter(im_g, ker_15');
im_g = im_g/256;

figure(1);
subplot(1,3,1);imagesc(im);caxis([0,255]);
subplot(1,3,2);imagesc(im_f);caxis([0,255]);
subplot(1,3,3);imagesc(im_g);caxis([0,255]);

im_ker = zeros(1,15);
im_ker(8) = 256;
figure(2);hold off;plot(ker_15, 'r');
for i = 1:12
    im_ker = imfilter(im_ker, [1,1,1]/3);
    figure(2);hold on;plot(im_ker, 'b');
end
