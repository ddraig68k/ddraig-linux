/*
 * $Id: parse_config.h,v 1.2 2003/10/22 14:15:55 janakj Exp $
 *
 * PERMISSIONS module
 *
 * Copyright (C) 2003 Mikl?s Tirp?k (mtirpak@sztaki.hu)
 *
 * This file is part of ser, a free SIP server.
 *
 * ser is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version
 *
 * For a license to use the ser software under conditions
 * other than those described here, or to purchase support for this
 * software, please contact iptel.org by e-mail at the following addresses:
 *    info@iptel.org
 *
 * ser is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef PARSE_CONFIG_H
#define PARSE_CONFIG_H 1

#include "rule.h"

rule *parse_config_file(char *filename);

#endif /* PARSE_CONFIG_H */
