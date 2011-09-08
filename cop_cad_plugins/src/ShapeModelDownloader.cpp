/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "ShapeModelDownloader.h"
#include "ShapeModel.h"
#include "DxfWriter.h"
#include "DxfReader.h"
#include "Signature.h"
#include "Class.h"
using namespace cop;


ShapeModelDownloader::ShapeModelDownloader()
{
}

ShapeModelDownloader::~ShapeModelDownloader(void)
{
}

/*
**	@(#) $Id: LoadToFile.c,v 1.7 2000/07/04 15:26:57 kahan Exp $
**
**	More libwww samples can be found at "http://www.w3.org/Library/Examples/"
**
**	Copyright © 1995-1998 World Wide Web Consortium, (Massachusetts
**	Institute of Technology, Institut National de Recherche en
**	Informatique et en Automatique, Keio University). All Rights
**	Reserved. This program is distributed under the W3C's Software
**	Intellectual Property License. This program is distributed in the hope
**	that it will be useful, but WITHOUT ANY WARRANTY; without even the
**	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
**	PURPOSE. See W3C License http://www.w3.org/Consortium/Legal/ for more
**	details.
**
**	Sample showing how to load a document and save it to local file
*/
extern "C"
{

#ifndef WIN32
#define HAVE_STRERROR
#include <wwwsys.h>
typedef struct in_addr {
  union {
      struct {
       u_char s_b1,s_b2,s_b3,s_b4;
      } S_un_b;
      struct {
         u_short s_w1,s_w2;
      } S_un_w;
      u_long S_addr;
    } S_un;
}_in_addr;

struct sockaddr_in{
   short sin_family;
   unsigned short sin_port;
   struct in_addr sin_addr;
   char sin_zero[8];
};
#else
#define HAVE_STRERROR
#define __MINGW32__
#endif


#include <WWWLib.h>			      /* Global Library Include file */
#include <WWWMIME.h>				    /* MIME parser/generator */
#include <WWWNews.h>				       /* News access module */
#include <WWWHTTP.h>				       /* HTTP access module */
#include <WWWFTP.h>
#include <WWWFile.h>
#include <WWWGophe.h>
#include "WWWInit.h"
}
#define APP_NAME		"GETTOOL"
#define APP_VERSION		"1.0"
#define DEFAULT_OUTPUT_FILE     "get.out"

int replace_str(char *str, const char *orig, const char *rep)
{
  int count = 0;
  char *p;
  char temp [4096];

  p = strstr(str, orig);
  while(p)  // Is 'orig' even in 'str'?
  {
      strcpy(temp, p+strlen(orig));
      sprintf(p, "%s%s", rep, temp);
      p = strstr(str, orig);
      count++;
  }
  return count;
}


int replace_fn_command(char *url, const char* rep)
{
  //int count = 0;
  char *p, *q;
  char temp [4096];

  p = strstr(url, "fn=");
  q = strstr(p, "&");
  if(p)  // Is 'orig' even in 'str'?
  {
      strcpy(temp, q);
      sprintf(p, "fn=%s%s", rep, temp);
  }
  return 1;
}

void DeleteFileIfExist(const char* outputfile2)
{
  FILE* ftest = fopen(outputfile2, "r");
  if(ftest != NULL)
  {

#ifdef WIN32
    replace_str(outputfile2, "/", "\\");
    std::string deletestring = std::string("del ") + std::string(outputfile2);
#else
    std::string deletestring = std::string("rm -rf ") + std::string(outputfile2);
#endif
    fclose(ftest);
    int result = system(deletestring.c_str());
    if(result != 0)
     printf("Unexpected result in system call: %s : %d", deletestring.c_str(), result);
  }
}

void RenameFiles(const char* inputname, char* output)
{

#ifdef WIN32
  char temp[4096];
  strcpy(temp, inputname);
  replace_str(temp, "/", "\\");
  std::string deletestring = std::string("rename ") + std::string(temp) + std::string(" ") + std::string(output);
#else
  std::string deletestring = std::string("mv ") + std::string(inputname) + std::string(" ") + std::string(output);
#endif
  printf("%s\n", deletestring.c_str());
  int i = system(deletestring.c_str());
  if(i != 0)
    printf("Unexpected result in system call: %s : %d", deletestring.c_str(), i);

}


PRIVATE int printer_links (const char * fmt, va_list pArgs)
{
    return (vfprintf(stdout, fmt, pArgs));
}

PRIVATE int tracer_links (const char * fmt, va_list pArgs)
{
    return (vfprintf(stderr, fmt, pArgs));
}

PRIVATE int terminate_handler_links (HTRequest * request, HTResponse * response,
			       void * param, int status)
{

    HTEventList_stopLoop();
    return 0;
}

int s_counter = 0;
char adresses_to_download[24][1024];

PRIVATE void foundLink (HText * text,
			int element_number, int attribute_number,
			HTChildAnchor * anchor,
			const BOOL * present, const char ** value)
{
    if (anchor) {
	/*
	**  Find out which link we got. The anchor we are passed is
	**  a child anchor of the anchor we are current parsing. We
	**  have to go from this child anchor to the actual destination.
	*/


    HTAnchor * dest = HTAnchor_followMainLink((HTAnchor *) anchor);
    char * address = HTAnchor_address(dest);

    BOOL found = NO;
    int cmp;
    int cnt;



    if(address == NULL)
        return;
    cmp = strncmp(address, "http://sketchup.google.com/3dwarehouse/download", strlen("http://sketchup.google.com/3dwarehouse/download"));
    if(cmp == 0)
    {
        SGML_dtd * dtd = HTML_dtd();
        HTTag * tag = SGML_findTag(dtd, element_number);
        found = NO;
        if(tag != NULL)
        {
          //char * tagname = HTTag_name(tag);
          int maxcnt = HTTag_attributes(tag);
          for (cnt=0; cnt<maxcnt; cnt++)
          {
              if (present[cnt])
              {
                  char * attrname = HTTag_attributeName(tag, cnt);
                  cmp = strcmp(attrname ? attrname : "<unknown>", "CLASS");
                  if(cmp == 0)
                  {
                      cmp = strcmp(value[cnt] ,"dwnld");
                      if(cmp == 0)
                      {
                          found = YES;
                          break;
                      }

                  }
              }
          }
        }
        if(found == YES)
        {
            HTPrint("Found link `%s\'\n", address);
            if(s_counter < 24)
            {
            strcpy(adresses_to_download[s_counter++], address);
            }
            else
            {
                HTPrint("Skip link `%s\'\n", address);
            }
        }

    }
    HT_FREE(address);
  }
}

int GetLinks(char * uri_in)
{
    char* uri = NULL;
    /* Create a new premptive client */
    HTProfile_newHTMLNoCacheClient ("ShowLinks", "1.0");

    /* Need our own trace and print functions */
    HTPrint_setCallback(printer_links);
    HTTrace_setCallback(tracer_links);

    /* Set trace messages and alert messages */
#if 0
    HTSetTraceMessageMask("sop");
#endif

    /* Add our own termination filter */
    HTNet_addAfter(terminate_handler_links, NULL, NULL, HT_ALL, HT_FILTER_LAST);

    /*
    ** Register our HTML link handler. We don't actually create a HText
    ** object as this is not needed. We only register the specific link
    ** callback.
    */
    HText_registerLinkCallback(foundLink);

    /* Setup a timeout on the request for 15 secs */
    HTHost_setEventTimeout(30000);

    /* Handle command line args */
    uri = HTParse(uri_in, NULL, PARSE_ALL);

    if (uri)
    {
        HTRequest * request = NULL;
        HTAnchor * anchor = NULL;
        BOOL status = NO;

        /* Create a request */
        request = HTRequest_new();
        if(request == NULL)
          return 0;
        /* Get an anchor object for the URI */
        anchor = HTAnchor_findAddress(uri);
        if(anchor == NULL)
          return 0;

        /* Issue the GET and store the result in a chunk */
        status = HTLoadAnchor(anchor, request);

        /* Go into the event loop... */
        if (status == YES) HTEventList_loop(request);

        /* We are done with this request */
        HTRequest_delete(request);
        HTProfile_delete();
    }
    else
    {
        return 0;
    }
    return 1;
}
/*

PRIVATE int printer (const char * fmt, va_list pArgs)
{
    return (vfprintf(stdout, fmt, pArgs));
}

PRIVATE int tracer (const char * fmt, va_list pArgs)
{
    return (vfprintf(stderr, fmt, pArgs));
}
*/
/*
**  We get called here from the event loop when we are done
**  loading. Here we terminate the program as we have nothing
**  better to do.
*/
int terminate_handler_my (HTRequest * request, HTResponse * response,
    		       void * param, int status)
{

    HTEventList_stopLoop();
    return 1;
}


void DownloadedFileName(char* buffer , const char* basename, int number)
{
    sprintf(buffer, "%s_%d.zip", basename, number);
}

std::vector<std::string> LoadFiles(int offset, std::string searchstring, std::string outputfile)
{
    //int		        arg = 0;
    int i;
    //char *              getme = NULL, getme2 = NULL;
    HTRequest *         request = NULL;
    std::vector<std::string> ret;

    char tempurl[1024];
    int start = 12;
    sprintf(tempurl, "http://sketchup.google.com/3dwarehouse/search?q=%s&start=%d", searchstring.c_str(), start);
    GetLinks(tempurl);

#ifdef WIN32
    /* Initiate W3C Reference Library with a client profile */
    HTProfile_newNoCacheClient(APP_NAME, APP_VERSION);
    /* Need our own trace and print functions */
    HTPrint_setCallback(printer);
    HTTrace_setCallback(tracer);

    /* Add our own filter to terminate the application */
    HTNet_addAfter(terminate_handler_my, NULL, NULL, HT_ALL, HT_FILTER_LAST);

    /* Set the timeout for how long we are going to wait for a response */
    HTHost_setEventTimeout(10000);
#endif

    for(i = 0; i < s_counter; i++)
    {
      int checker = 0;
      char outputfile2[1024];

      request = HTRequest_new();

      replace_str(adresses_to_download[i], "&amp;", "&");
      checker = replace_str(adresses_to_download[i], "rtyp=s6", "rtyp=zip") + replace_str(adresses_to_download[i], "rtyp=s7", "rtyp=zip") + + replace_str(adresses_to_download[i], "rtyp=s&", "rtyp=zip&");
      if(checker == 0) /*Wrong type, dont download*/
          continue;
      DownloadedFileName(outputfile2 , outputfile.c_str(), i);
      DeleteFileIfExist(outputfile2 );
      /* Start the load */
      replace_fn_command(adresses_to_download[i], outputfile.c_str());
#ifdef WIN32
      HTPrint("link a %s\n", adresses_to_download[i]);
      if (HTLoadToFile(adresses_to_download[i], request, outputfile2) != YES) {
        HTPrint("Can't open output file\n");
      }
      else
      {
        ret.push_back(outputfile2);
      }

      /* Go into the event loop... */
      HTEventList_loop(request);
  /* Delete our request again */
      HTRequest_delete(request);
#else
    char  command[4096];
    sprintf(command, "wget \"%s\" -O %s", adresses_to_download[i], outputfile2);
    printf("Sending to system: %s\n", command);
    int result = system(command);
    if(result != 0)
      printf("Unexpected result in system call: %s : %d", command, result);


    FILE* ftest = fopen(outputfile2, "r");
    if (ftest == NULL) {
      printf("Can't open output file\n");
    }
    else
    {
      fclose(ftest);
      ret.push_back(outputfile2);
    }


#endif
    }
    /* Delete our profile */
#ifdef WIN32
    HTProfile_delete();
#endif


    return ret;
}

Descriptor*  ShapeModelDownloader::FindModels(std::string searchString)
{
  ShapeModel* sm = NULL;
  s_counter = 0;
  std::vector<std::string> count_files = LoadFiles(0, searchString,searchString);
  printf("Got %ld results from the first run searching for %s\n", count_files.size(), searchString.c_str());
  if(count_files.size() < 8) /*Google return 12 hits at max, and expects multiple of 12 as start index*/
  {
    std::vector<std::string> tmp =  LoadFiles(12, searchString,searchString);
    count_files.insert(count_files.end(), tmp.begin(), tmp.end());
  }
  Class* cl = NULL;
  cl = new Class();
  std::stringstream sttemp;
  sttemp << searchString << "_CAD";
  cl->SetName(sttemp.str());

  for(size_t i = 0; i< count_files.size(); i++)
  {
    std::string outputfile = count_files[i];
    std::string unzip_cmd;
    char modelname[4096];
    unzip_cmd = std::string("unzip -j -o -d unzipped ") + outputfile;
    /**Delete unspecific modelname in the dae file before extracting the next one*/
    sprintf(modelname, "%s_%ld.dae", searchString.c_str(), i);
#ifdef WIN32
    DeleteFileIfExist("unzipped/*.*");
#else
    DeleteFileIfExist("unzipped");
#endif
    int result = system(unzip_cmd.c_str());
    if(result != 0)
      printf("Unexpected result calling: %s : %d\n", unzip_cmd.c_str(), result);
#ifdef WIN32
    RenameFiles("unzipped/*.dae", modelname);
    sprintf(modelname, "unzipped/%s_%d.dae", searchString.c_str(), i);
#else
    result  = system("ls unzipped/*.dae");
    if(result != 0)
      printf("Unexpected result calling: %s : %d\n", unzip_cmd.c_str(), result);
    sprintf(modelname, "unzipped/%s_%ld.dae", searchString.c_str(), i);
    RenameFiles("unzipped/*.dae", modelname);
#endif
    sprintf(modelname, "unzipped/%s_%ld.dae", searchString.c_str(), i);
    FILE *f = fopen(modelname, "r");
    if(f == NULL)
    {
      printf("Model not found:\n   %s\n", modelname);
      DeleteFileIfExist("unzipped/*.*");
      if(strlen(outputfile.c_str()) >= 4096)
        throw "Error with to long filenames";
      strcpy(modelname,outputfile.c_str());
      DeleteFileIfExist(modelname);
      continue;
    }
    fclose(f);
    Matrix m = IdentityMatrix(4);
    int points;
    Mesh_t mesh = ReadMesh(modelname, 1.0, m, points);
    /*Clean up before saving the file:
      dae format extracts to:
          doc.kml
          texture.txt
          models.dae (=modelname)
       and was saved to
       %s_%d.zip (=outputfile)
    */
    DeleteFileIfExist(modelname);
    if(strlen(outputfile.c_str()) >= 4096)
        throw "Error with to long filenames";
    strcpy(modelname,outputfile.c_str());
    DeleteFileIfExist(modelname);
    DeleteFileIfExist("unzipped/*.*");
    sprintf(modelname, "%s_%ld.dxf", searchString.c_str(), i);
    dxfwriter::WriteMesh(mesh, modelname);
    printf("Testing: Create ShapeModel\n");
    if(cl == NULL)
      printf("Class NULL?!??\n");
    else{
        cl->GetName();
        printf("Cl accesl worked\n");
    }
    sm = new ShapeModel(new Class(cl->GetName(), cl->m_ID));
    sm->m_initializationLevel = 0.0;
    if(sm == NULL)
      printf("ShapeModel == NULL?!??\n");
    else{
        sm->Save();
        printf("sm accesss worked\n");
    }
    ShapeModelParamSet* pm = new ShapeModelParamSet(NULL,0.0,0.0,0.0); /**TODO: Add Calib*/
    sm->SetShapeModelParamSet(pm, modelname, false);
  }
  return sm;
}
std::string FindOpenClass(Signature& object)
{
  for(unsigned int i = 0; i < object.CountClasses(); i++)
  {
    Class* cl = object.GetClass(i);
    if(object.GetClass(i)->GetName().compare("Cluster") != 0)
      return cl->GetName();
  }
  return  "";
}

Descriptor* ShapeModelDownloader::Perform(std::vector<Sensor*> sensors, RelPose* pose, Signature& object, int &numOfObjects, double& qualityMeasure)
{
  return FindModels(FindOpenClass(object));
}

double ShapeModelDownloader::CheckSignature(const Signature& object, const std::vector<Sensor*> &sensors)
{
  if(object.CountClasses() > object.CountElems() + 1)
  {
     printf("ShapeModelDownloader::CheckSignature: enough classes\n");
    if(object.GetElement(0, DESCRIPTOR_SHAPE) != NULL)
      return 0.0;
    return 1.0;
  }
  return 0.0;
}

XMLTag* ShapeModelDownloader::Save()
{
  XMLTag* tag = new XMLTag(XML_NODE_SHAPEMODELDOWNLOADER);
  return tag;
}
