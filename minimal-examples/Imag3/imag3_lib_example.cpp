
#include <Coyot3pp/Imag3/ImageContent/ImageContentTestImage.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentImgSequence.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentComposition.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentCvVideocap.hpp>
#include <Coyot3pp/Imag3/ImageContent/ImageContentDisplayImage.hpp>

#ifndef IMAG3_EXAMPLES_DIRECTORY
#define IMAG3_EXAMPLES_DIRECTORY ""
#endif

namespace cai = coyot3::av::image;
namespace ct = coyot3::tools;
bool dostuff = true;

void on_control_c(int sig)
{
  CLOG_WARN("ending")
  dostuff = false;
}
int main(int argv, char **argc)
{
  cai::ImageContentTestImage img;
  cai::ImageContentParams params;
  cai::ImageContentCvVideocap vid("yo", "/dev/video0");
  params.width(500);
  params.height(500);
  params.base_path("");
  params.show_preview(false);
  params.show_name(true);

  img.set_params(params);
  img.name("testSource");
  img.id(2);
  img.debug_level_set(10);
  cai::ImageContentImgSequence seq;
  cai::ImageContentImgSequenceParamsImage item;

  seq.set_params(params);
  seq.name("sequence");
  params.width(600);
  params.height(500);
  
  seq.id(1);
  item.name("uno");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-01.png");
  seq.params_imgsec().sequence().push_back(item);
  item.name("dos");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-02.jpeg");
  seq.params_imgsec().sequence().push_back(item);
  item.name("tres");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-03.jpg");
  seq.params_imgsec().sequence().push_back(item);
  item.name("cuatro");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-04.jpg");
  seq.params_imgsec().sequence().push_back(item);
  item.name("cinco");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-05.jpg");
  seq.params_imgsec().sequence().push_back(item);
  item.name("seis");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-06.jpg");
  seq.params_imgsec().sequence().push_back(item);
  item.name("siete");
  item.path(IMAG3_EXAMPLES_DIRECTORY "/coyot3-07.png");
  seq.params_imgsec().sequence().push_back(item);
  seq.params_imgsec().interval(3000);
  seq.params_imgsec().mode(cai::ec::ImageContentImgSequenceMode::SEQUENCE_NORMAL);

  img.Init();
  img.Start();

  seq.Init();
  seq.Start();

  cai::ImageContentDisplayImage disp;
  disp.set_params(params);
  disp.name("display");
  disp.preserve_aspect_ratio(true);
  disp.Init();
  disp.Start();

  cai::ImageContentComposition tree;
  params.width(600);
  params.height(800);

  vid.set_params(params);
  vid.name("video");
  vid.id(0);
  vid.show_preview(false);
  vid.connection_retry_interval(5000);
  vid.Init();
  vid.Start();

  params.show_name(false);
  tree.set_params(params);

  tree.name("tree");

  tree.debug_level_set(5);

  tree.source_attach(vid);
  tree.source_attach(img);
  tree.source_attach(seq);

  tree.source_attach(disp);
  CLOG_INFO(" inicializando tree : ")
  tree.Init();
  CLOG_INFO(" arrancando tree : ")
  tree.Start();
  tree.show_preview(true);
  CLOG_INFO("entrando al loop")
  int64_t ts = coyot3::tools::get_current_timestamp();
  int state{0};
  while (dostuff)
  {
    usleep(75000);
    // seq.prepare_output();
    //  vid.shift_source(
    //    static_cast<int>(ct::generate_natural_number(12,-12)),
    //    static_cast<int>(ct::generate_natural_number(12,-12)));
    //CLOG_INFO("preparando output");
    tree.prepare_output();

    int64_t now = coyot3::tools::get_current_timestamp();
    if ((now - ts) > 2000)
    {
      ts = now;
      CLOG_INFO("main - current state=" << state)
      switch (state)
      {
      case 0:
        tree.image_focus(1);
        tree.organization(cai::ec::ImcTreeOrganization::TARGETED_LIST_AUTO);
        break;
      case 1:
        tree.image_focus(2);
        break;
      case 2:
        tree.image_focus(3);
        break;
      case 3:
        tree.organization(cai::ec::ImcTreeOrganization::MOSAIC);
        break;
      case 4:
        tree.organization(cai::ec::ImcTreeOrganization::SINGLE_SOURCE);
        tree.image_focus(1);
        break;
      case 5:
        tree.image_focus(2);
        break;
      case 6:
        tree.image_focus(3);
        break;
      case 7:
        tree.image_focus(4);
      default:
        state = -1;
      }
      state++;
    }
  }

  img.End(true);
  seq.End(true);
}