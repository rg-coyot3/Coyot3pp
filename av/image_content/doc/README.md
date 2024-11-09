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

> Check the minimal example to see how to use.

### `ImageContent` 

`ImageContent(const std::string& name)` is the base class, with the goal to be able to easily invoke effects to the final product of an image.

To configure the `ImageContent` use a `ImageContentParams` instance:
* `width`, `height`.
* `show_preview(bool)`
* `show_name(bool)`
* `effects` : is of type `ImageContentEffectsParams`
  * `rotation(int)`
  * `transparency(double)`
  * `src_shifting_x(int)`, `src_shifting_y(int)`
  * `black_and_white(bool)`
  * `border_width(int)`.


The `ImageContent` class inherits from the `coyot3::mod::ModuleBase`, meaning that is designed to include (if needed) initialization, starting, pausing, stopping and ending methods that need to be configured at the constructor of the class. 

The `ImageContent` class is taught to be the base for image processing units that are going to be used by other objects, simplifying its use by:

* configuring the instance
* initializing the instance
* starting the instance...

> *that simple*

The `ImageContent` class and child classes need `Init()` *initialization* after the configuration of the class.


### `ImageContentCvVideoCap`.

To be able to use the benefits of the `cv::VideoCap` source. 
Can be invoked used `ImageContentCvVideocap(std::string name, std::string video_source)`

### `ImageContentImgSequence`

To be able to include sequence of images.

### `ImageContentDisplayImage` 

To create a source from the desktop source.

### `ImageContentComposition`

To be able to add multiple sources and create a single one. The instance will automatically create a mosaic with it. It is possible to select a specific source to see it bigger.


