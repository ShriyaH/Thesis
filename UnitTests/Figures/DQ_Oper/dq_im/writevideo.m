video = VideoWriter('yourvideo.avi'); %create the video object
open(video); %open the file for writing
for ii=1:30 %where N is the number of images
  str = num2str(ii);  
  I = imread('str.jpg'); %read the next image
  writeVideo(video,I); %write the image to file
end
close(video);