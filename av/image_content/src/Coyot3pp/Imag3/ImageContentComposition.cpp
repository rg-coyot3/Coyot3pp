#include <Coyot3pp/Imag3/ImageContent/ImageContentComposition.hpp>



namespace cw = coyot3::mod;

namespace coyot3{
namespace av{
namespace image{


  CYT3MACRO_enum_class_definitions(
    ImcTreeOrganization
    ,
      , MOSAIC             
      , TARGETED_LIST_RIGHT
      , TARGETED_LIST_DOWN 
      , TARGETED_LIST_UP   
      , TARGETED_LIST_LEFT 
      , TARGETED_LIST_AUTO
      , CASCADE            
      , FROM_SOURCE_PARAMS 
      , SINGLE_SOURCE
  )


  ImageContentComposition::ImageContentComposition(const std::string& name)
  :ImageContent(name)
  ,sources()
  ,organization_(ec::ImcTreeOrganization::MOSAIC){
    class_name("imc-composition");
    log_info("constructor");

    add_task_init(std::bind(&ImageContentComposition::task_init,this),true);
    add_task_start(std::bind(&ImageContentComposition::task_start,this),true);
    add_task_pause(std::bind(&ImageContentComposition::task_pause,this),true);
    add_task_stop(std::bind(&ImageContentComposition::task_stop,this),true);
    add_task_end(std::bind(&ImageContentComposition::task_end,this),true);
  }

  ImageContentComposition::~ImageContentComposition(){
    log_warn("destructor");
  }

  bool ImageContentComposition::erase_all(){
    SourcesMap::iterator it;
    for(it = sources.begin(); it != sources.end();++it){
      log_warn(o() << "erase-all : erasing [" << it->second->name() << "]");
      delete it->second;
    }
    log_warn("image-content-composition : erase-all : done");
    sources.clear();
    return true;
  }

  int ImageContentComposition::source_attach(const ImageContent* img){
    CLOG_EVALUATION(img != nullptr,
    "image-content-composition : image-add",
    "adding [" << img->name() << "]",
    "INPUT IS VOID!")
    if(img == nullptr)return -1;
    int index = static_cast<int>(sources.size() + 1);
    image_focus(index);
    sources.insert(std::make_pair(index,(ImageContent*)img));
    switch(state()){
      case cw::ec::CytModuleState::STARTED:
      case cw::ec::CytModuleState::PAUSED:
      case cw::ec::CytModuleState::STOPPED:
        calculate_params_for_sources();
        break;
    } 
    return int(sources.size());
  }
  int ImageContentComposition::source_attach(ImageContent& img){    
    return source_attach(&img);
  }

  ImageContent* ImageContentComposition::source_get(int id){
    SourcesMap::iterator it = sources.find(id);
    if(it == sources.end()){
      return nullptr;
    }
    return it->second;
  }
  ImageContent* ImageContentComposition::source_detach(int i){
    ImageContent* ret;
    SourcesMap::iterator it = sources.find(i);
    bool allgood = true;
    if(it == sources.end())return nullptr;
    ret = it->second;
    sources.erase(it);
    allgood &= calculate_params_for_sources();
    CLOG_EVALUATION(allgood 
    ,"image-content-composition : source-add"
    ,"sources params updated"
    ,"alerts updating sources params")
    return ret;
  }
  ImageContent* 
  ImageContentComposition::source_detach(const ImageContent* i){
    ImageContent* ret;
    SourcesMap::iterator it, rem = sources.end();

    for(it == sources.begin();it != sources.end();++it){
      if(i == it->second)rem = it;
    }
    if(rem == sources.end())return nullptr;
    ret = rem->second;
    sources.erase(rem);
    CLOG_EVALUATION(calculate_params_for_sources()
    ,"image-content-composition : source-add"
    ,"sources params updated"
    ,"alerts updating sources params")
    return ret;
  }

  ec::ImcTreeOrganization 
  ImageContentComposition::organization(ec::ImcTreeOrganization org){
    organization_ = org;
    log_info(o() << "organization  : set [" << organization_ 
      << "] while [" << state() << "]");
    switch(state()){
      case cw::ec::CytModuleState::STARTED:
      case cw::ec::CytModuleState::PAUSED:
        calculate_params_for_sources();
        break;
      case cw::ec::CytModuleState::CREATED:
      case cw::ec::CytModuleState::INITIALIZED:
      case cw::ec::CytModuleState::END_OF_LIFE:
      case cw::ec::CytModuleState::STOPPED:
      case cw::ec::CytModuleState::MODULE_ERROR:
      default:
        log_debug(7, "organization : nothing to do");
    }
    return organization_;
  }

  ec::ImcTreeOrganization 
  ImageContentComposition::organization() const{
    return organization_;
  }


  bool 
  ImageContentComposition::calculate_params_for_mosaic(int indexExclude ,
      int mosX , 
      int mosY ,
      int mosW , 
      int mosH ){
      return calculate_params_for_mosaic_v2(mosX, mosY, mosW, mosH);
  }

  bool 
  ImageContentComposition::calculate_params_for_mosaic_v1(
      int indexExclude ,
      int mosX , 
      int mosY ,
      int mosW , 
      int mosH ){
      
    //si los parámetros vienen por defecto, entonces se cubre la pantalla entera
    if((mosW <= 0) || (mosH <= 0)){
      mosX = 0;
      mosY = 0;
      mosW = params_.width();
      mosH = params_.height();
    }
    int totalSurface;
    int unitSurface, unitHeight, unitWidth, numCols, numRows;
    
    int offsetX, offsetY,offsetXLastRow;
    bool configFound;
    
    // primero averiguamos el aspect ratio.
    double aspectRatio  =   static_cast<double>(params_.width()) 
                          / static_cast<double>(params_.height());

    int totalMosaicImages = 0;

    for(SourcesMap::iterator it = sources.begin() ; it != sources.end();++it){
      if((indexExclude != -1) && (indexExclude == it->second->id())){
        continue; //no cuentes con esta imagen.
      }
      if(it->second->params().is_static()){
        continue; //no cuentes tampoco con las imágenes estáticas.
      }
      ++totalMosaicImages;
    }
    CLOG_DEBUG(7,"image-content-composition : calc-mosaic(" 
      << mosX << "," << mosY << "," << mosW << "," << mosH << ") : imgs = [" 
      << totalMosaicImages << "] - asr(" << aspectRatio << ")")
    //estos son los offsets iniciales de las coordenadas relativas...
    // offsetX = std::floor(     static_cast<double>(mosX) 
    //                           /static_cast<double>(params_.width()));
    // offsetY = std::floor(     static_cast<double>(mosY) 
    //                           /static_cast<double>(params_.height()));
    

    /*
      el algoritmo se basa en lo siguiente:
        - como máximo, el área resultante de cada celda del mosaico será igual
          al área de la superficie a mostrar dividido por el número de 
          componentes del mosaico.
        - cada celda del mosaico tendrá las mismas proporciones que el 
          contenedor.
        - haz en bruteforce: el resultado de multiplicar el número de celdas
          que caben en horizontal, por el número de celdas que caben en vertical
          es al menos igual al número de fuentes de imagenes totales. SI a la
          hora de calcular la base y la altura con el área que existe en un 
          momento dado, no puedes colocarlas todas, reduce el tamaño del área...
          (no es muy sofisticado PERO!, por el momento
          voy a tratar de hacerlo así dado que de todas formas casi nunca va a 
          haber ni muchas celdas, ni el tamaño de la imagen va a ser enorme...)  
    */      
    totalSurface = mosW * mosH;
    unitSurface  = totalSurface    / totalMosaicImages;  
                
    do{
      
      unitHeight   = std::floor(std::sqrt(static_cast<double>(unitSurface) / aspectRatio));
      unitWidth    = std::floor(aspectRatio * static_cast<double>(unitHeight));

      numCols = std::floor(mosW / unitWidth);
      numRows = std::floor(mosH / unitHeight);

      CLOG_DEBUG(7,"----C:" << numCols << ",F:" << numRows << ";uH:" << unitHeight 
        << ",uW:" << unitWidth << "; provisional cols=" << numCols 
        << ", rows=" << numRows)

      if((numCols * numRows) >= totalMosaicImages){
        configFound = true;
      }else{
        configFound = false;
        unitSurface-=10;
      }
      // CLOG_EVALUATION(configFound,
      // "image-content-composition : calc-mosaic(" 
      //   << mosX << "," << mosY << "," << mosW << "," << mosH << ") : imgs = [" 
      //   << totalMosaicImages << "] : " << name(),
      // "OK to place images for a surface [" << unitSurface << "](" << unitWidth 
      //   << " x " << unitHeight << ")(cols=" << numCols << ", rows=" << numRows 
      //   << ")",
      // "surface [" << unitSurface << "] is still too much ")



    }while(configFound == false);

    //caso particular
    if(aspectRatio>=1){
      //la imagen es más ancha que alta
      while(totalMosaicImages <= (numCols * (numRows-1)))
        --numRows;
    }else{
      while(totalMosaicImages <= ((numCols - 1) * numRows))
        --numCols;
    }


    //ahora a calcular los offsets...

    offsetX += mosX  + std::floor( (mosW - (numCols * unitWidth ) -1 ) / 2);
    offsetY += mosY  + std::floor( (mosH - (numRows * unitHeight) - 1) / 2);

          // las celdas se van colocando de izquierda a derecha y de arriba a abajo.
      // la última fila es probable que no se complete así que se van a dividir
      //   la dimensión de los huecos entre dos y se añadirá al offset de la 
      //    última fila. el número de columnas en la última fila es el residuo
      //    de dividir el número de fuentes entre el número de columnas máximas
      //    por fila. si ese resíduo es cero, entonces la última fila no tiene
      //    huecos.
    offsetXLastRow = mosX + std::floor(((totalMosaicImages % numCols) * unitWidth)/2);
    log_info(o() << "offset-x-last-row = " << offsetXLastRow);
    //inicialización del punto de impresión con los offsets iniciales
    int currentX = offsetX, currentY = offsetY;

    //... un par de variables de apoyo...
    double x0, y0, xf, yf;
    //... yyy empieza la fiefshta...
    SourcesMap::iterator it = sources.begin();
    for(int indexRow = 0; indexRow < numRows; ++indexRow){
      currentX = offsetX;
      if(indexRow == (numRows-1)){
        //si es la última fila, añade el offset X de la última fila
        currentX+=offsetXLastRow;
      }

      log_info(o() << "------iterando, " << indexRow << ", " << it->first );
      //el bucle anidado tiene el condicionante adiccional de no pasarse de 
      // rosca con el puntero/iterador sobre el mapa.
      for(int indexCol = 0; 
              ((indexCol < numCols) && (it != sources.end())); 
              ++indexCol, ++it){
        log_info(o() << "iC: " << indexCol);
        if(it->second->params().is_static()){
          log_info("es estática");
           continue;} //las estáticas no se 
                                                      //  reconfiguran
        if((it->second->id() == indexExclude) && (indexExclude != -1)){
          log_info(o() << "se excluye la imagen [" << it->second->id() << "]");
          continue; // se exclude la imagen indicada.
        }
          

        x0 =    static_cast<double>(currentX) 
              / static_cast<double>(mosW);
        
        xf =    static_cast<double>(currentX + unitWidth)  
              / static_cast<double>(mosW);

        y0 =    static_cast<double>(currentY) 
              / static_cast<double>(mosH);
        
        yf =    static_cast<double>(currentY + unitHeight) 
              / static_cast<double>(mosH);
        
        it->second->set_relative_position(x0,y0,xf,yf);
        log_info(o() << "***** " << x0 << "," << y0 << ";" << xf << "," << yf);
        it->second->cross_image_dimensions_with_rels(
                          mosW,
                          mosH,
                          it->second->preserve_aspect_ratio());
        
        currentX+=unitWidth;
        
      }
      currentY+=unitHeight;
    }
    // ... y llegados a este punto... el mosaico está organizado.
    return true;
  }


  bool ImageContentComposition::calculate_params_for_mosaic_v2(
      int indexExclude ,
      int mosX , 
      int mosY ,
      int mosW , 
      int mosH ){
    log_debug(5,o() << "calc-params-mosaic : " << mosX << ", " << mosY 
      << " => " << mosW << ", " << mosH << " ; indexExcl=" << indexExclude);

    if(
          (mosX <0) 
      || (mosY < 0) 
      || (mosW <= 0) 
      ||(mosW > params_.width()) 
      || (mosH <= 0) 
      || (mosH > params_.height())){
      log_debug(3,o() << "calc-params-for-mosaic-v2 : " << name() 
        << " : setting all image area");
      mosX = 0;
      mosY = 0;
      mosW = params_.width();
      mosH = params_.height();
    }

    int totalMosaicImages;
    std::vector<SourcesMap::iterator> sourcesToAdd;
    for(SourcesMap::iterator it = sources.begin() ; it != sources.end();++it){
      if((indexExclude != -1) && (indexExclude == it->first)){
        continue; //no cuentes con esta imagen.
      }
      if(it->second->params().is_static()){
        continue; //no cuentes tampoco con las imágenes estáticas.
      }
      sourcesToAdd.push_back(it);
    }

    totalMosaicImages = static_cast<int>(sourcesToAdd.size());

    int numCols, numRows;
    double aspectRatio = static_cast<double>(mosW) / static_cast<double>(mosH);

    numCols = numRows = 1;
    log_info(o() << "calc-params-for-mosaic-v2 : " << name() << 
      "calculating grid to contain [" << totalMosaicImages 
      << "] : aspect-ratio=" << aspectRatio);
    if(aspectRatio >= 1){
      while(
        (numCols * static_cast<int>(std::round(static_cast<double>(numCols) 
                        / aspectRatio))) < totalMosaicImages)
      {
        
        ++numCols;
      }
      numRows = static_cast<int>(std::round(static_cast<double>(numCols) / aspectRatio));
      
    }else{
      while(
        (numRows * std::round(static_cast<double>(numRows) * aspectRatio)) 
          < totalMosaicImages)
      {
        ++numRows;
      }
      numCols = static_cast<int>(std::round(static_cast<double>(numRows) 
                                  * aspectRatio));
    }
    log_info(o() << "calc-params-for-mosaic-v2 : " << name() 
    << " : estimated a container grid of [ " << numCols << " x " 
    << numRows << " ]");

    int cellWidth(std::floor(mosW / numCols)), 
        cellHeight(std::floor(mosH / numRows));
    

    int excess = (numCols * numRows) - totalMosaicImages;

    int lastRowOffsetX = std::floor(excess * cellWidth / 2);

    int indexCol{0}, indexRow{0};
    
    
    double x0,y0,xf,yf,pcw,pch,xi,yi;
    pcw = static_cast<double>(cellWidth) / static_cast<double>(params_.width());
    pch = static_cast<double>(cellHeight) / static_cast<double>(params_.height());
    std::vector<SourcesMap::iterator>::iterator it = sourcesToAdd.begin();
    xi = static_cast<double>(mosX) / static_cast<double>(params_.width());
    yi = static_cast<double>(mosY) / static_cast<double>(params_.height());
    for(indexRow = 0; indexRow < numRows ; ++indexRow){
      double pcOffsetXLastLine {0.};
      
      if(indexRow == (numRows - 1)){
        //última línea
        pcOffsetXLastLine = (static_cast<double>(lastRowOffsetX) / static_cast<double>(params_.width()));
      }
      
      y0 = yi + static_cast<double>(indexRow * cellHeight) / static_cast<double>(params_.height());
      yf = y0 + pch;

      for(indexCol = 0; 
          (indexCol < numCols) && (it != sourcesToAdd.end()); 
          ++indexCol, ++it){
        bool par {(*it)->second->preserve_aspect_ratio()};
        x0 = xi + pcOffsetXLastLine + (static_cast<double>(indexCol * cellWidth) 
                                  / static_cast<double>(mosW));
        
        xf = x0 + pcw;
        
        (*it)->second->set_relative_position(x0,y0,xf,yf,par);
        (*it)->second->cross_image_dimensions_with_rels(params_.width(),
                                                        params_.height(),
                                                        par);
      }
      
    }
    return true;
  }
  
  bool ImageContentComposition::calculate_params_for_single_source(){
    SourcesMap::iterator it = sources.find(image_focus_id_);
    if(it == sources.end()){
      return false;
    }
    it->second->set_relative_position(0.,0.,1.,1.);
    it->second->cross_image_dimensions_with_rels(params_.width(),
                                          params_.height(),
                                          it->second->preserve_aspect_ratio());
    return true;
  }

  bool
  ImageContentComposition::calculate_params_for_mosaic_v0(){
    //REMI LUNAY - square-root system
    int mosaic_divisor = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(sources.size()))));
  

    double proportion_out = (static_cast<double>(params_.width()) 
                            / static_cast<double>(params_.height()));
    double width_out, heigh_out;
    
    if(proportion_out >= 1.0){
      //la imagen resultante es más ancha que alta
      width_out = std::floor((params_.width() - 1) / mosaic_divisor);
      heigh_out = std::floor(params_.width()  / proportion_out);
    }else{
      //la imagen resultante es más alta que ancha
      heigh_out = std::floor((params_.height() - 1) / mosaic_divisor);
      width_out = std::floor(params_.height() * proportion_out);
    }

    log_info(o() << "calculate-params-for-mosaic : calculated "
      "parameters for each image : [" << width_out << " x " 
      << heigh_out << "]");


    return false;
  }

  /**
   * @brief de una parte está la zona del mosaico, y de otra está la imagen en
   *        grande.
   * 
   * @return true 
   * @return false 
   */

  bool 
  ImageContentComposition::calculate_params_for_focused_image(){
    if(sources.size() == 0){
    log_warn("calculate-params-for-focused-cfg- : THERE ARE NO SOURCE IMAGES!");
    return false;
    }
    int mosX,mosY,mosW,mosH,indexExcl;
    log_debug(3,o() << "calculate-params-for-focused-cfg- : setting [" 
      << organization() << "]");
    SourcesMap::iterator it = sources.find(image_focus_id_);
    if(it == sources.end()){
      log_warn(o() << "calculate-params-for-focused-cfg : index image [" 
        << image_focus_id_ << "] NOT FOUND!");
    }
    if(organization() == ec::ImcTreeOrganization::TARGETED_LIST_AUTO){
      if(params_.aspect_ratio() >= 1.0){
        organization_ = ec::ImcTreeOrganization::TARGETED_LIST_RIGHT;
      }else{
        organization_ = ec::ImcTreeOrganization::TARGETED_LIST_DOWN;
      }

    }
    log_info(o() << "calculate-params-for-focused-cfg- : ["
    << image_focus_id_ << "](" << it->second->name() 
    << ") is focused at [" << organization() 
    << "]");
    switch(organization()){
      case ec::ImcTreeOrganization::TARGETED_LIST_UP:
        mosX = 0;
        mosY = 0;
        mosW = params_.width();
        mosH = std::floor(static_cast<double>(params_.height()) / 3.0);
        it->second->set_relative_position(
          0.,
          1.0 / 3.0,
          1.0,
          1.0
        ); 
        
        
        break;
      case ec::ImcTreeOrganization::TARGETED_LIST_DOWN:
        mosX = 0;
        mosY = std::floor(static_cast<double>(2 * params_.height()) / 3.0);
        mosW = params_.width();
        mosH = std::floor(static_cast<double>(params_.height()) / 3.0);
        it->second->set_relative_position(
          0,
          0,
          1.0,
          2.0 / 3.0
        ); 
        break;
      case ec::ImcTreeOrganization::TARGETED_LIST_LEFT:
        mosX = 0;
        mosY = 0;
        mosW = std::floor(static_cast<double>(params_.height()) / 3.0);
        mosH = params_.height();
        it->second->set_relative_position(
          1.0/3.0,
          0,
          1.0,
          1.0
        ); 
        break;
      case ec::ImcTreeOrganization::TARGETED_LIST_RIGHT:
        mosX = std::floor(static_cast<double>(2 * params_.width()) / 3.0);
        mosY = 0;
        mosW = std::floor(double(params_.width()) / 3.0);
        mosH = params_.height();
        it->second->set_relative_position(
          0,
          0,
          2.0/3.0,
          1.0
        ); 
        break;
      default:
        CLOG_WARN("image-content-composition : calculat")
        return false;

    }
    it->second->cross_image_dimensions_with_rels(
          params_.width(),params_.height(), it->second->preserve_aspect_ratio());

    calculate_params_for_mosaic_v2(image_focus_id_,
                                        mosX,mosY,mosW,mosH);
  

    return true;
  }



bool ImageContentComposition::calculate_params_for_sources(){
  CLOG_INFO("image-content-composition : calculate-params-for-sources : [" 
    << organization() << "]")
  switch(organization()){
    case ec::ImcTreeOrganization::TARGETED_LIST_AUTO:
    case ec::ImcTreeOrganization::TARGETED_LIST_DOWN:
    case ec::ImcTreeOrganization::TARGETED_LIST_UP:
    case ec::ImcTreeOrganization::TARGETED_LIST_RIGHT:
    case ec::ImcTreeOrganization::TARGETED_LIST_LEFT:
      return calculate_params_for_focused_image();
      break;
    case ec::ImcTreeOrganization::SINGLE_SOURCE:
      return calculate_params_for_single_source();
      break;
    case ec::ImcTreeOrganization::CASCADE:
    case ec::ImcTreeOrganization::FROM_SOURCE_PARAMS:
    case ec::ImcTreeOrganization::UNKNOWN_OR_UNSET:
    case ec::ImcTreeOrganization::MOSAIC:
    default:
      return calculate_params_for_mosaic_v2(-1);
      break;
  }
  return false;
}



int ImageContentComposition::image_focus(int index){
  if(sources.find(index) == sources.end())return -1;
  bool updateProduct= false;
  if(image_focus_id_ != index){
    image_focus_id_ = index;
    switch(organization()){
      case ec::ImcTreeOrganization::SINGLE_SOURCE:
        calculate_params_for_single_source();
        break;
      case ec::ImcTreeOrganization::TARGETED_LIST_AUTO:
      case ec::ImcTreeOrganization::TARGETED_LIST_UP:
      case ec::ImcTreeOrganization::TARGETED_LIST_DOWN:
      case ec::ImcTreeOrganization::TARGETED_LIST_LEFT:
      case ec::ImcTreeOrganization::TARGETED_LIST_RIGHT:
        calculate_params_for_focused_image();
        break;
    }
  }
  return index;
}
int ImageContentComposition::image_focus() const{
  return image_focus_id_;
}


bool ImageContentComposition::imageprocess_paste_src_image(
  int id,int xi,int yi,int xf,int yf)
{
  SourcesMap::iterator it = sources.find(id);
  if(it == sources.end()){
    log_warn(o() << "source id does not exist : [" << id << "]");
    return false;
  }
  ImageContent* ptr = it->second;
  bool opres = true;
  int sw, sh;
  try{
    std::lock_guard guard(mtx_source_);
    std::lock_guard guardsrc(ptr->getMutexRef());
    cv::Mat& buffer = ptr->getImageRef();
    if((xi == -1) ||(yi == -1) ||(xf == -1) ||(yf== -1 )){
      xi = static_cast<int>(std::floor(static_cast<double>(image_source.cols) 
                                        * ptr->params().rxi() ));
      yi = static_cast<int>(std::floor(static_cast<double>(image_source.rows) 
                                        * ptr->params().ryi() ));
      log_debug(7,o() << "imgproc-paste-src-image : id[" << id 
        << "] - calculating params from source.");
    }
    xf = xi + ptr->get_width() - 1;
    yf = yi + ptr->get_height() - 1;
    

    buffer.copyTo(image_source(cv::Rect(xi,yi,ptr->get_width(),ptr->get_height())));

  }catch(const cv::Exception& e){
    log_warn(o() << "error pasting image : id [" << id << "] "
    "from ((" << image_source.cols << "," << image_source.rows << ") · ((" 
      << ptr->params().rxi() << "," << ptr->params().ryi() << "),("
      << ptr->params().rxf() << "," << ptr->params().ryf() << "))"
    "to   (" << xi << "," << yi << " => " << ptr->get_width() << "," << ptr->get_height() << ") . "
    "Exception(" << e.what() << ") : "
    "section-dim(" << sw << "," << sh << ")");
    opres = false;
  }catch(...){
    log_warn(o() << "error pasting image : id [" << id << "] "
    "from ((" << image_source.cols << "," << image_source.rows << ") · ((" 
      << ptr->params().rxi() << "," << ptr->params().ryi() << "),("
      << ptr->params().rxf() << "," << ptr->params().ryf() << "))"
    "to   (" << xi << "," << yi << " => " << xf << "," << yf << ") . "
    "Exception( UNKNOWN ) "
    "section-dim(" << sw << "," << sh << ")");

    opres = false;
  }
  
  //cv::imshow(ptr->name()+"**",image_source);

  return opres;
}



bool ImageContentComposition::imageprocess_paste_sources(){
  SourcesMap::iterator it;
  int doneok = 0;
  img_source_initialize();
  for(it = sources.begin();it != sources.end(); ++it){
    if(imageprocess_paste_src_image(it->first))++doneok;
  }
  return sources.size() == doneok;
}

bool ImageContentComposition::imageprocess_paste_focused_image(){
  SourcesMap::iterator it;
  return imageprocess_paste_src_image(image_focus());
}






bool ImageContentComposition::task_init(){
  
  log_info("init : begin");
  SourcesMap::iterator it;
  size_t doneok = 0;
  bool   allgood = true;
  log_info("init : initializing sources");
  allgood &= calculate_params_for_sources();
  for(it = sources.begin();it != sources.end();++it){
    if(it->second->created()){
      if(it->second->Init()== true)++doneok;
      else log_warn(o() << "init : error initializing (" << it->second->name() << ")");
    }else{
      ++doneok;
    }
  }
  allgood &= (doneok == sources.size());

  allgood &= ImageContent::task_init();
  log_eval(allgood,
  "init : done ok",
  "init : alerts at initialization");
  
  return allgood;
}

bool ImageContentComposition::task_start(){
   
  log_info("start : begin");
  SourcesMap::iterator it;
  size_t doneok = 0;
  bool   allgood = true;
  allgood &= calculate_params_for_sources();
  log_info("start : starting sources");
  for(it = sources.begin();it != sources.end();++it){
    if(it->second->initialized()){
      if(it->second->Start()== true)++doneok;
      else log_info(o() << "start : error start (" << it->second->name() << ")");
    }else{
      ++doneok;
    }
  }
  allgood &= (doneok == sources.size());
  CLOG_EVALUATION(allgood,
  "image-content-composition : start : " << name(),
  "done ok",
  "alerts while starting sources")
  
  return allgood;
}
bool ImageContentComposition::task_pause(){
  SourcesMap::iterator it;
  size_t doneok = 0;
  bool   allgood = true;
  log_info("pause : pausing sources");
  for(it = sources.begin();it != sources.end();++it){
    if(it->second->Pause()== true)++doneok;
  }
  allgood &= (doneok == sources.size());
  allgood &= ImageContent::Pause();
  CLOG_EVALUATION(allgood,
  "image-content-composition : pause : " << name(),
  "done ok",
  "errors while pausing sources")
  
  return allgood;

}

bool ImageContentComposition::task_stop(){
  log_info("stop : begin");
  SourcesMap::iterator it;
  size_t doneok = 0;
  bool   allgood = true;
  log_info("stop : stopping sources");
  for(it = sources.begin();it != sources.end();++it){
    if(it->second->Stop()== true)++doneok;
  }
  allgood &= (doneok == sources.size());
  allgood &= ImageContent::Stop();
  CLOG_EVALUATION(allgood,
  "image-content-composition : stop : " << name(),
  "done ok",
  "errors while stopping sources")
  return allgood;
}

bool ImageContentComposition::task_end(){
  
  SourcesMap::iterator it;
  size_t doneok = 0;
  bool   allgood = true;
  log_info("end : ending sources");
  for(it = sources.begin();it != sources.end();++it){
    if(it->second->End()== true)++doneok;
  }
  allgood &= (doneok == sources.size());
  allgood &= ImageContent::Stop();
  sources.clear();
  CLOG_EVALUATION(allgood,
  "image-content-composition : end() : " << name(),
  "done ok",
  "errors ending sources")
  return allgood;
}

bool ImageContentComposition::prepare_output(){
  SourcesMap::iterator it;
  bool allgood = true;
  if(organization() == ec::ImcTreeOrganization::SINGLE_SOURCE){
    sources.find(image_focus_id_)->second->prepare_output();
    allgood &= imageprocess_paste_focused_image();
  }else{
    for(it= sources.begin(); it != sources.end(); ++it){
      allgood &= it->second->prepare_output();
    }
    allgood &= imageprocess_paste_sources();  
  }
  
  allgood &= ImageContent::prepare_output();
  return allgood;
}






}
}
}