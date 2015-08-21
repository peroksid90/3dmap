/**
@mainpage Модуль под ROS для построения 3D модели окружающей среды
\section mainpage_overview Введение
Этот модуль предназначен для построения 3D модели окружающей среды при помощи стерео-пары и двух лидаров.
В модуле используются две сторонние библиотеки - OpenCV и pointclouds.
OpenCV - это широко известная и проверенная временем библиотека для обработки изображений.
В этот модуль включены 2 примера из исходных кодов OpenCV(stereo_match.cpp и stereo_calib.cpp) предназначеных для калибровки
и получения облака точек.
Наиболее важными функциями, используемыми в этих примерах, являются:
findChessboardCorners - для поиска на изображениях внутренних углов калибровочной шахматной доски
stereoCalibrate - получить калибровочные параметры камер(intrinsic, extrinsic) параметры
stereoRectify - computes the rectification transformation for a stereo camera from its intrinsic and extrinsic parameters.
stereoRectify так же заполняет важный параметр - матрицу Q, которая используется функцией
reprojectImageTo3D - получающей непосредственно Z координату.

Для дополнительной информации по OpenCV в контексте нашего модуля мы рекомендуем:
1. Книга Learning OpenCV Computer Vision in C++ with the OpenCV Library, главы Camera Models and Calibration и Projection and 3D Vision.
2. Книга Practical OpenCV, глава 3D Geometry and Stereo Vision
Отдельно рекомендуем ознакомится с этой документацией:
http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

Pointclouds - библиотека для обработки облаков точек. Она позволяет объединять облака точек в единое целое.
Фильтровать их, распознавать геометрические фигуры и многое другое.
Предполагается, что вы располагаете облаком точек(XYZ) координат в каком либо формате(ply, pcd). Либо,
вы можете загрузить облака точек из любого текстового файла, вручную заполняя облако точек.
\code
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
cloud->push_back(pcl::PointXYZ(x, y, z));
\endcode
Эта библиотека так-же располагает бинарными файлами(при установке через пакетный менеджер, напр. apt-get они расположены в стандартных директориях
/usr/bin/) начинающихся с префикса pcl_. Среди них есть очень полезные инструменты:
pcl_pcd2ply - конвертация из pcd в ply.
pcl_viewer - визуализировать облако точек
Главным модулем pointclouds для нас является модуль registratioin.
Он содержит алгоритмы для объединения облаков точек.
В pointclouds есть два подхода к registration:
1. Используя Iterative Closest Points алгоритм, который методом подбора пытается трансформировать целевое облако к исходному
и наложить их с наименьшей ошибкой. Документация: http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point
Пример в функции template<typename PointType> typename pcl::PointCloud<PointType>::Ptr SumClouds(const std::vector<typename pcl::PointCloud<PointType>::ConstPtr>& clouds)
2. Найти keypoints и features. Использовать их для объединения облаков. Пример в pcl_registration.cpp


\section mainpage_build Сборка проекта
Предполагается, что на сервере установлена Ubuntu и установлены пакеты ROS по
<a href="http://wiki.ros.org/indigo/Installation/Ubuntu">документации ROS indigo</a>.
Выполняя установку пакетов, желательно выбрать вариант Desktop-Full Install(вы его увидите, следуя документации по ссылке)
Для сборки модуля используется catkin workspace. Прежде чем использовать модуль, настройте у себя
catkin workspace, следуя этому <a href="http://wiki.ros.org/catkin/Tutorials/create_a_workspace">уроку</a>.
После настройки workspace скопируйте этот модуль(целиком папку env_3dmap) в ~/catkin_ws/src/ (эти инструкции даются с учетом того,
что вы настроили workspace в соответствии с уроком по ссылке выше, т.е. в этом случае ваш workspace должен называтся именно catkin_ws)
Пример:
\code
cp -r ~/env_3dmap ~/catkin_ws/src/
\endcode
Теперь проект готов для сборки. Что бы собрать проект, используйте следующие команды:
\code
cd ~/catkin_ws
catkin_make
\endcode

После сборки, вы можете запускать бинарные файлы модуля, пример:
\code
cd ~/catkin_ws/devel/lib/env_3dmap
./stereo_match
\endcode

\section mainpage_struct Общая структура модуля
\code
env_3dmap/
├── package.xml
├── CMakeLists.txt
└── src
    ├── env_3dmap_node.cpp
    ├── pcl_registration.cpp
    ├── stereo_calib.cpp
    ├── stereo_match.cpp
    └── utils.h
\endcode
Файлы package.xml и CMakeLists.txt являются элементами сборочной системы caktin. Они содержат необходимые зависимости для модуля.
За подробностями обращайтесь к документации <a href="http://wiki.ros.org/catkin/Tutorials">catkin</a>.

\subsection mainpage_struct_env_3dmap_node env_3dmap_node.cpp
Предполагается, что этот модуль должен включать в себя код из всех остальных частей проекта(калибровку камеры, построение disparity map)
но в процессе разработки было принято решение временно разбить все эти задачи на отдельные исполняемые файлы.
В этом модуле происходит подписка на <a href="http://wiki.ros.org/Topics">Topics</a> с левой и правой камеры, получение изображений с них
и преобразование изображений в формат OpenCV.

Дальнейшее реализовано в остальных исполняемых файлах: используя инструменты OpenCV нужно получить XYZ облако точек на основе калибровочных данных камер и полученное облако точек
обработать используя библиотеку <a href="http://pointclouds.org/">pointclouds</a>.

Конечная цель - получить последовательность облак точек(сделанных на основе изображений какого-либо объекта окружающей среды, например стола с разных ракурсов)
и объединить их в одно облако точек - что бы полученая модель объекта(напр. стола) была наиболее полной и ее можно было смотерть с разных ракурсов в графических редакторах.

В идеале модуль должен работать следующим образом - непрерывно получать пары изображений по Topics, преобразовывать их в облака точек и "наслаивать" на построенную ранее
3D модель(если таковая уже имелась).
\subsection mainpage_struct_pcl_registration pcl_registration.cpp
Исполняемый файл на основе данного исходного кода принимает два облака точек(первое из которых получено например, при повороте головы робота на 30 градусов, а второе - на 33 градуса)
и совмещает их в единое облако точек, в computer vision это называется <a href="http://pointclouds.org/documentation/tutorials/registration_api.php">registration</a>
\subsection mainpage_struct_stereo_calib stereo_calib.cpp
Выполняет калибровку камер, подробней в \ref stereo_calib
\subsection mainpage_struct_stereo_match stereo_match.cpp
Выполняет получение облака точек(XYZ координат) по паре изображений, подробней в \ref stereo_match
\subsection mainpage_struct_utils utils.h
Функции для пользователей(нахождение keypoints, features, трансформации облаков точек и т.д.).
\section mainpage_algorithm Алгоритм работы с модулем
Настроить у робота руками в голове диафрагму и светочувствительность на желаемое расстояние. На этом расстоянии разместить
калибровочную доску. Выполнить 30-60 снимков этой доски в самых различных положениях(смещая, сдвигая, поворачивая).
Запустить \ref stereo_calib который выдаст калибровочные параметры.
Запустить \ref stereo_match который выдаст облако точек.
Использовать функцию int utils::load_from_xyz(const std::string& file, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
что бы загрузить в PCL это облако точек. И уже дальше в коде можно его визуализировать и сохранить в ply или pcd формате.
Получить второе облако точек с другого ракурса по этим же шагам.
Полученные 2 облака точек передать в \ref pcl_registration который визуализирует результат совмещения облаков. Но он принимает их только в формате ply
Если у вас ряд облаков точек, то нужно использовать функцию template<typename PointType> typename pcl::PointCloud<PointType>::Ptr SumClouds(const std::vector<typename pcl::PointCloud<PointType>::ConstPtr>& clouds)
Для нее нет отдельного бинарного файла, необходимо самостоятельно реализовать ее использование.
\warning stereo_match не возвращает цвет в облаке точек.

Рекомендую использовать сторонюю утилиту StereoVision:
https://github.com/erget/stereovision
После ее установки будет доступно два исполняемых файла. Нужно использовать вначале этот
calibrate_cameras
\code
usage: calibrate_cameras [-h] [--rows ROWS] [--columns COLUMNS]
                         [--square-size SQUARE_SIZE] [--show-chessboards]
                         input_folder output_folder
--rows - число внутренних углов на шах.доске по ширине
--columns - число внутренних углов на шах.доске по высоте
--square-size - размер квадратика шах.доски, указывается в любой единице измерения(см. или мм) - на нашей доске было 3.4см
input_folder - должен содержать изоббражения шахматной доски. Изображения должны называтся leftX.tiff , rightX.tiff
Напр. left01.tiff, right01.tiff и т.д.
l1.tiff или r1.tiff - не сработают.
output_folder - папка, в которую он сохранит расчитанные калибровочные параметры
\endcode

После этого, нужно использовать
\code
images_to_pointcloud
usage: images_to_pointcloud [-h] [--use_stereobm] [--bm_settings BM_SETTINGS]
                            calibration left right output
calibration - папка с калибровочными параметрами(output_folder)
left - левое изображение
right - правое изображение
output - расчитаное облако точек.
\endcode

Расчитаное облако точек(output) дальше можно передавать в pcl_registration
\section mainpage_feature Future work
Нужно разобраться почему камеры так плохо калибруются. Очень часто, выставив диафрагму и сделав 50-60 снимков не достигается результата
До сих пор не ясно из-за чего так происходит.
После решения проблемы с калибровкой, нужно детально разобратся с алгоритмами построения disparity map.
Поскольку disparity map как правило получается низкого качества. И из за этого дальнейшее облако точек будет сильно зашумленное.
Проблема с disparity map исследуется уже около 30-лет, разработано множество алгоритмов. Те, которые работают лучше всего - очень медленные
Решение проблем с disparity map и калибровочными параметрами автоматически позволит получить очень хорошее облако точек.
И на этом этапе нужно экспериментировать с этими облаками, пытаясь их слить в единое целое, выбрав наиболее подходящий алгоритм из registration модуля в pointclouds.
\section mainpage_test Производительность и технические данные

Модули протестированы на камерах Basler acA640-90gc http://www.baslerweb.com/ru/produkty/matrichnye-kamery/ace/aca640-90gc
и лидаре Hokuyo URG-04LX-UG01
Запускались модули на ноутбуке Acer Aspire V3-772
Процессор Intel(R) Core(TM) i7-4702MQ CPU @ 2.20GHz 8 ядер.
Оперативная память: 24 Гб
- stereo_calib.cpp длительность выполнения зависит от количества переданных фотографий, обработка 60-ти фотографий занимает 3 мин.
- stereo_match.cpp длительность сильно зависит от параметра --scale, если выставлять его на единицу, то длительность 30 сек в среднем.
- pcl_registration.cpp длительность 3 мин.
- env_3dmap_node.cpp тестировался только на Gazebo, длительность 5 сек.

*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <string.h>
using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
/**
@brief Класс для обработки видео-потока с топиков ROS
@details Подписывается на топики видео-камер, синхронизирует получаемые данные с обоих камер и конвертирует их в OpenCV изображения.
*/
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_left;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_right;
public:
  /**
  @brief Конструктор
  @details Подписывается на топики с камер, создает объект sync для синхронизации получаемых видео-потоков и регистрирует коллбэк,
  который будет вызыватся при получении новых данных с топиков.
  */
  ImageConverter()
    : it_(nh_)
  {
    ROS_INFO("ImageConverter constructor");
    image_sub_left.subscribe(nh_, "/wide_stereo/left/image_rect_color", 1);
    image_sub_right.subscribe(nh_, "/wide_stereo/right/image_rect_color", 1);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(image_sub_left, image_sub_right, 1);
    sync.registerCallback(boost::bind(ImageConverter::imageCb, _1, _2));
    cv::namedWindow(OPENCV_WINDOW);
    ros::spin();
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  /**
  @brief Коллбэк для обработки изображений с камеры
  @param msg_left [in] Изображение с левой камеры
  @param msg_right [in] Изображение с правой камеры
  @details В тот момент, когда новые данные приходят по топикам, вызывается этот коллбэк заряженый полученными изображениями с левой и правой камер.
  Эти изображения уже синхронизированы между собой.
  В коллбэке необходимо вести всю дальнейшую обработку этих изображений - построение disparity map, получение облакак точек, объединение его с предыдущими облаками точек
  */
  static void imageCb(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
  {
    ROS_INFO("imageCb callback");
    cv_bridge::CvImagePtr cv_ptr_left;
    cv_bridge::CvImagePtr cv_ptr_right;
    try
    {
      cv_ptr_left = cv_bridge::toCvCopy(msg_left, sensor_msgs::image_encodings::BGR8);
      cv_ptr_right = cv_bridge::toCvCopy(msg_right, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //TODO получить disparity map по cv_ptr_left и cv_ptr_right а из нее облако точек(XYZ)
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "env_3dmap_node");
  ImageConverter ic;
  return 0;
}
