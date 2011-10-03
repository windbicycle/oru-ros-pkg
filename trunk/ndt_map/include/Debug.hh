/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdio.h>

//default debug level is 0 / off
//of course if a developer specifies negative level, it will be on. please don't :)

#ifndef DBG_LVL
#define DBG_LVL 0
#endif

#ifndef NO_ERR
//error macro. prints the error statement and location, exits
#define ERR(...) fprintf(stderr, "[%s:%d] %s: ", \
	                     __FILE__, __LINE__, __FUNCTION__); \
		 fprintf(stderr, __VA_ARGS__); exit(-1); 
#else
#define ERR(...) ;
#endif

//debug macro. if the current output level (lvl) is less then the DBG_LVL 
//outputs line/file where statement occurs + statement in printf syntax
#define DBG(lvl,...) if((lvl<DBG_LVL)) { \
			fprintf(stderr, "[%s:%d] %s: ", \
	                     __FILE__, __LINE__, __FUNCTION__); \
			fprintf(stderr, __VA_ARGS__); }

#endif //__DEBUG_H__

