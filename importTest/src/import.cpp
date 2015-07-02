#include <resource_retriever/retriever.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
  resource_retriever::Retriever r;
  resource_retriever::MemoryResource resource;

  try
  {
     resource = r.get("file:///home/nathaniel/Desktop/test.txt"); 
  }
  catch (resource_retriever::Exception& e)
  {
    ROS_ERROR("Failed to retrieve file: %s", e.what());
    return 1;
  }

  FILE* f = fopen("out.txt", "w");
  fwrite(resource.data.get(), resource.size, 1, f);
  fclose(f);
  
  ROS_INFO("Wrote data from package:/home/nathaniel/Desktop/test.txt to out.txt");
}
