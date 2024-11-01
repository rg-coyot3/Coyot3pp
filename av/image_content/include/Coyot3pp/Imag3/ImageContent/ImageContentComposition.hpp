#pragma once



#include "ImageContent.hpp"

namespace coyot3{
namespace av{
namespace image{
  
  
  CYT3MACRO_enum_class_declarations(
    ImcTreeOrganization
    ,
      , MOSAIC              = 0
      , TARGETED_LIST_RIGHT = 1
      , TARGETED_LIST_DOWN  = 2
      , TARGETED_LIST_UP    = 3
      , TARGETED_LIST_LEFT  = 4
      , TARGETED_LIST_AUTO  = 5
      , CASCADE             = 6
      , FROM_SOURCE_PARAMS  = 7
      , SINGLE_SOURCE       = 8
  )

  class ImageContentComposition
  : public ImageContent{
    public:

    typedef std::map<int,ImageContent*>           SourcesMap;

            ImageContentComposition(const std::string& name = std::string());
    virtual ~ImageContentComposition();


    
    /**
     * @brief attaches a source to the composition. 
     * 
     * @param img 
     * @return int : -1 on error, acquired index if success.
     */ 
    int           source_attach(const ImageContent* img);
    int           source_attach(      ImageContent& img);


    virtual bool prepare_output() override;

    /**
     * @brief detaches from stack AND frees mem
     * 
     * @param i 
     * @return true 
     * @return false 
     */
    ImageContent*         source_detach(int i);
    /**
     * @brief detaches from stack but does not free mem
     * 
     * @param i 
     * @return true 
     * @return false 
     */
    ImageContent*         source_detach(const ImageContent* i);
    ImageContent*         source_get(int id);
    bool                  erase_all();

    ec::ImcTreeOrganization   organization(ec::ImcTreeOrganization org);
    ec::ImcTreeOrganization   organization() const;


    int                  image_focus(int index);
    int                  image_focus() const;

  protected:
    SourcesMap                sources; 

    ec::ImcTreeOrganization   organization_;

    bool    task_init();
    bool    task_start();
    bool    task_pause();
    bool    task_stop();
    bool    task_end();

    bool calculate_params_for_sources();




    /**
     * @brief calcula los parámetros para crear un mosaico
     * 
     * @param indexExclude : índice de la imagen a excluir
     * @param mosX : x inicial de la zona mosaico
     * @param mosY : y inicial de la zona mosaico
     * @param mosW : ancho de la zona de mosaico
     * @param mosH : altura de la zona de mosaico
     * @return true 
     * @return false 
     */

    bool calculate_params_for_mosaic(
      int indexExclude = -1,
      int mosX = -1, 
      int mosY = -1,
      int mosW = -1, 
      int mosH = -1);

    bool calculate_params_for_mosaic_v1(
      int indexExclude = -1,
      int mosX = -1, 
      int mosY = -1,
      int mosW = -1, 
      int mosH = -1);

    bool calculate_params_for_mosaic_v2(
      int indexExclude = -1,
      int mosX = -1, 
      int mosY = -1,
      int mosW = -1, 
      int mosH = -1);

    bool sources_organize_for_mosaic();

        bool calculate_params_for_mosaic_v0();
    
    
    bool calculate_params_for_focused_image();

      bool sources_organize_for_focused();
    
    bool calculate_params_for_single_source();
    
    bool imageprocess_update_sources_target_dimensions();
      int      image_focus_id_;

    bool imageprocess_paste_src_image(int srcId, 
        int xi = -1, 
        int yi = -1, 
        int xf = -1, 
        int yf = -1);   //locks source-image-mutex
      
    bool imageprocess_paste_sources();
    bool imageprocess_paste_focused_image();
  

    
  private:






  };


}
}
}