/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "PCCConfigurationFileParser.h"

using namespace std;
using namespace pcc;

ostream& PCCErrorMessage::error( const std::string& errMessage, const std::string& where ) {
  is_errored_ = true;
  if ( !where.empty() ) {
    cerr << "\n\nError: " << errMessage << "\n" << where << endl;
  } else {
    cerr << "\n\nError: " << errMessage << endl;
  }
  return cerr;
}

std::ostream& PCCErrorMessage::warn( const std::string& warningMessage, const string& where ) {
  if ( !where.empty() ) {
    cerr << " Warning:" << warningMessage << "\n" << where << endl;
  } else {
    cerr << " Warning: " << warningMessage << endl;
  }
  return cerr;
}

void PCCConfigurationFileParser::scanLine( std::string& line, KeyValMaps& key_val_maps ) {
  // Ignore comment lines starting with "**********"
  if ( line.rfind( "**********", 0 ) == 0 ) { return; }
  std::map<std::string, std::string> key_val_map;
  std::string                        spKey[3] = {"Occupancy", "Geometry", "Attribute"};
  if ( line.find( "#" ) != std::string::npos ) { return; }  // skip comment line
  size_t start = line.find_first_not_of( " \t\n\r" );
  if ( start == string::npos ) { return; }  // blank line
  line.erase( std::remove( line.begin(), line.end(), ' ' ), line.end() );
  line += " ";
  while ( line.size() > 1 ) {
    std::string keyName = "Not Set", keyValue = "";
    size_t      curPos = line.find_first_of( "=" );
    if ( curPos != std::string::npos ) {
      keyName = line.substr( 0, curPos );
    } else {
      for ( auto& e : spKey ) {
        size_t found = line.find( e );
        if ( found != std::string::npos ) {
          key_val_map.insert( std::pair<std::string, std::string>( e, "" ) );
          key_val_maps.push_back( key_val_map );
          return;
        }
      }
    }
    if ( !validKey( keyName ) ) {
      error( where() ) << "Unknown Key: " << keyName << endl;
      return;
    }
    auto endPos = line.find_first_of( ", ", curPos );
    if ( endPos != std::string::npos ) {
      keyValue = line.substr( curPos + 1, endPos - curPos - 1 );
      // ajt::how to check valid key value in case of missing ,?
    } else {
      error( where() ) << " missing ',' " << endl;
      return;
    }
    key_val_map.insert( std::pair<std::string, std::string>( keyName, keyValue ) );
    line.erase( 0, endPos + 1 );
  }
  key_val_maps.push_back( key_val_map );
}

void PCCConfigurationFileParser::scanStream( std::istream& in, KeyValMaps& key_val_maps ) {
  do {
    linenum_++;
    std::string line;
    getline( in, line );
    scanLine( line, key_val_maps );
  } while ( !!in );
}

bool PCCConfigurationFileParser::parseFile( std::string& fileName, KeyValMaps& key_val_maps ) {
  name_    = fileName;
  linenum_ = 0;
  std::ifstream cfrStream( name_.c_str(), std::ifstream::in );
  if ( !cfrStream ) {
    error( name_ ) << "Failed to Open : " << name_ << endl;
    return false;
  }
  scanStream( cfrStream, key_val_maps );
  return true;
}

bool PCCConfigurationFileParser::validKey( std::string& name ) {
  bool found = false;
  for ( auto key : keyList_ ) {
    if ( !name.compare( key ) ) { found = true; }
  }
  return found;
}
