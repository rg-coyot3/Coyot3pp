# `ImageContent`

## INTRODUCTION

`ImageContent` and its inherited classes are a wrapper done over OpenCV to be able to easily "transport" an image to a `cv::Mat` file format from external video-sources.

* NEED

The need is pretty simple: from an *agent* or *manager* I need to be able to be able to create an instance of a connector to an image source. Usually this configuration comes in a form of JSON. I need to create a mosaic with that image and eventually be able to target that image as the main one, showing the rest of the images as a sequence in one side of the resulting mosaic.

I don't know how many sources I will have... and I don't know what kind of sources. I just know that sometimes it may be a webcam, sometimes a video image, some times it may be the "current" main screen... I also don't know witch are the dimensions of the resulting image.

I need them to be working in the background of the invoking *manager*... and I need them to be stable...

Eventually I will need to make some treatment to the image, as putting some kind of title, or the timestamp of the last capture, and give it some effects if available.

* SOLUTION

As a solution, it came to me to use a wrapper to `cv::Mat` and use the avantages of *OpenCV*. I decided that during my works, I would first *transport* all possible sources to OpenCV, to finally *render* them to the desired destination. *The first destination I had in mind is to create an RTSP server*.



