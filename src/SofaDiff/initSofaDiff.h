#ifndef INITSOFADIFF_H
#define INITSOFADIFF_H

#include <sofa/config.h>

#ifdef SOFA_BUILD_SOFADIFF
#define SOFA_SOFADIFF_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_SOFADIFF_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

/** mainpage
This is the main page of the doxygen documentation for SofaDiff.
 */

#endif
