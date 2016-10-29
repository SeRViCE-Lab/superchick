/*
*    This code grabs the pcd file tarred in clouds.tar
*	 It is useful for simulating vicon in case of a demo
*	 
*	Author: Olalekan Ogunmolu
*   Date: October 28, 2016
*   
*   Lab Affiliation: Gans' Lab, UT Dallas
*/


#include <pcl/pcl_config.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/io/tar.h>

#ifdef _WIN32
# include <io.h>
# include <windows.h>
# define pcl_open                    _open
# define pcl_close(fd)               _close(fd)
# define pcl_lseek(fd,offset,origin) _lseek(fd,offset,origin)
#else
# include <sys/mman.h>
# define pcl_open                    open
# define pcl_close(fd)               close(fd)
# define pcl_lseek(fd,offset,origin) lseek(fd,offset,origin)
# ifndef _PCD_IO_H
# 	include <pcl/io/pcd_io.h>
# endif
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// GrabberImplementation //////////////////////
struct pcl::PCDGrabberBase::PCDGrabberImpl
{
  PCDGrabberImpl (pcl::PCDGrabberBase& grabber, const std::string& pcd_path, float frames_per_second, bool repeat);
  PCDGrabberImpl (pcl::PCDGrabberBase& grabber, const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat);
  void trigger ();
  void readAhead ();
  
  // TAR reading I/O
  int openTARFile (const std::string &file_name);
  void closeTARFile ();
  bool readTARHeader ();

  pcl::PCDGrabberBase& grabber_;
  float frames_per_second_;
  bool repeat_;
  bool running_;
  std::vector<std::string> pcd_files_;
  std::vector<std::string>::iterator pcd_iterator_;
  TimeTrigger time_trigger_;

  pcl::PCLPointCloud2 next_cloud_;
  Eigen::Vector4f origin_;
  Eigen::Quaternionf orientation_;
  bool valid_;

  // TAR reading I/O
  int tar_fd_;
  int tar_offset_;
  std::string tar_file_;
  pcl::io::TARHeader tar_header_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
};

///////////////////////////////////////////////////////////////////////////////////////////
pcl::PCDGrabberBase::PCDGrabberImpl::PCDGrabberImpl (pcl::PCDGrabberBase& grabber, const std::string& pcd_path, float frames_per_second, bool repeat)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , running_ (false)
  , pcd_files_ ()
  , pcd_iterator_ ()
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), boost::bind (&PCDGrabberImpl::trigger, this))
  , next_cloud_ ()
  , origin_ ()
  , orientation_ ()
  , valid_ (false)
  , tar_fd_ (-1)
  , tar_offset_ (0)
  , tar_file_ ()
  , tar_header_ ()
{
  pcd_files_.push_back (pcd_path);
  pcd_iterator_ = pcd_files_.begin ();
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::PCDGrabberBase::PCDGrabberImpl::PCDGrabberImpl (pcl::PCDGrabberBase& grabber, const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat)
  : grabber_ (grabber)
  , frames_per_second_ (frames_per_second)
  , repeat_ (repeat)
  , running_ (false)
  , pcd_files_ ()
  , pcd_iterator_ ()
  , time_trigger_ (1.0 / static_cast<double> (std::max (frames_per_second, 0.001f)), boost::bind (&PCDGrabberImpl::trigger, this))
  , next_cloud_ ()
  , origin_ ()
  , orientation_ ()
  , valid_ (false)
  , tar_fd_ (-1)
  , tar_offset_ (0)
  , tar_file_ ()
  , tar_header_ ()
{
  pcd_files_ = pcd_files;
  pcd_iterator_ = pcd_files_.begin ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::PCDGrabberBase::PCDGrabberImpl::readAhead ()
{
  PCDReader reader;
  int pcd_version;

  // Check if we're still reading files from a TAR file
  if (tar_fd_ != -1)
  {
    if (!readTARHeader ())
      return;
    valid_ = (reader.read (tar_file_, next_cloud_, origin_, orientation_, pcd_version, tar_offset_) == 0);
    if (!valid_)
      closeTARFile ();
    else
    {
      tar_offset_ += (tar_header_.getFileSize ()) + (512 - tar_header_.getFileSize () % 512);
      int result = static_cast<int> (pcl_lseek (tar_fd_, tar_offset_, SEEK_SET));
      if (result < 0)
        closeTARFile ();
    }
  }
  // We're not still reading from a TAR file, so check if there are other PCD/TAR files in the list
  else
  {
    if (pcd_iterator_ != pcd_files_.end ())
    {
      // Try to read in the file as a PCD first
      valid_ = (reader.read (*pcd_iterator_, next_cloud_, origin_, orientation_, pcd_version) == 0);

      // Has an error occured? Check if we can interpret the file as a TAR file first before going onto the next
      if (!valid_ && openTARFile (*pcd_iterator_) >= 0 && readTARHeader ())
      {
        tar_file_ = *pcd_iterator_;
        valid_ = (reader.read (tar_file_, next_cloud_, origin_, orientation_, pcd_version, tar_offset_) == 0);
        if (!valid_)
          closeTARFile ();
        else
        {
          tar_offset_ += (tar_header_.getFileSize ()) + (512 - tar_header_.getFileSize () % 512);
          int result = static_cast<int> (pcl_lseek (tar_fd_, tar_offset_, SEEK_SET));
          if (result < 0)
            closeTARFile ();
        }
      }

      if (++pcd_iterator_ == pcd_files_.end () && repeat_)
        pcd_iterator_ = pcd_files_.begin ();
    }
    else
      valid_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::PCDGrabberBase::PCDGrabberImpl::readTARHeader ()
{
  // Read in the header
  int result = static_cast<int> (::read (tar_fd_, reinterpret_cast<char*> (&tar_header_), 512));
  if (result == -1)
  {
    closeTARFile ();
    return (false);
  }

  // We only support regular files for now. 
  // Addional file types in TAR include: hard links, symbolic links, device/special files, block devices, 
  // directories, and named pipes.
  if (tar_header_.file_type[0] != '0' && tar_header_.file_type[0] != '\0')
  {
    closeTARFile ();
    return (false);
  }

  // We only support USTAR version 0 files for now
  if (std::string (tar_header_.ustar).substr (0, 5) != "ustar")
  {
    closeTARFile ();
    return (false);
  }

  if (tar_header_.getFileSize () == 0)
  {
    closeTARFile ();
    return (false);
  }

  tar_offset_ += 512;

  return (true);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::PCDGrabberBase::PCDGrabberImpl::closeTARFile ()
{
  pcl_close (tar_fd_);
  tar_fd_ = -1;
  tar_offset_ = 0;
  memset (&tar_header_.file_name[0], 0, 512);
}

///////////////////////////////////////////////////////////////////////////////////////////
int
pcl::PCDGrabberBase::PCDGrabberImpl::openTARFile (const std::string &file_name)
{
  tar_fd_ = pcl_open (file_name.c_str (), O_RDONLY);
  if (tar_fd_ == -1)
    return (-1);

  return (0);
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::PCDGrabberBase::PCDGrabberImpl::trigger ()
{
  if (valid_)
    grabber_.publish (next_cloud_,origin_,orientation_);

  // use remaining time, if there is time left!
  readAhead ();
}

///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// GrabberBase //////////////////////
pcl::PCDGrabberBase::PCDGrabberBase (const std::string& pcd_path, float frames_per_second, bool repeat)
: impl_ (new PCDGrabberImpl (*this, pcd_path, frames_per_second, repeat))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::PCDGrabberBase::PCDGrabberBase (const std::vector<std::string>& pcd_files, float frames_per_second, bool repeat)
: impl_ (new PCDGrabberImpl (*this, pcd_files, frames_per_second, repeat))
{
}

///////////////////////////////////////////////////////////////////////////////////////////
pcl::PCDGrabberBase::~PCDGrabberBase () throw ()
{
  stop ();
  delete impl_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::PCDGrabberBase::start ()
{
  if (impl_->frames_per_second_ > 0)
  {
    impl_->running_ = true;
    impl_->time_trigger_.start ();
  }
  else // manual trigger
    impl_->trigger ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::PCDGrabberBase::stop ()
{
  if (impl_->frames_per_second_ > 0)
  {
    impl_->time_trigger_.stop ();
    impl_->running_ = false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::PCDGrabberBase::trigger ()
{
  if (impl_->frames_per_second_ > 0)
    return;
  impl_->trigger ();
}

///////////////////////////////////////////////////////////////////////////////////////////
bool pcl::PCDGrabberBase::isRunning () const
{
  return (impl_->running_);
}

///////////////////////////////////////////////////////////////////////////////////////////
std::string 
pcl::PCDGrabberBase::getName () const
{
  return ("PCDGrabber");
}

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::PCDGrabberBase::rewind ()
{
  impl_->pcd_iterator_ = impl_->pcd_files_.begin ();
}

///////////////////////////////////////////////////////////////////////////////////////////
float pcl::PCDGrabberBase::getFramesPerSecond () const
{
  return (impl_->frames_per_second_);
}

///////////////////////////////////////////////////////////////////////////////////////////
bool pcl::PCDGrabberBase::isRepeatOn () const
{
  return (impl_->repeat_);
}
