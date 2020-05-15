/* Copyright (C) 1989-2003 Free Software Foundation, Inc.
   This file contains the GNU History Library (the Library), a set of
   routines for managing the text of previously typed lines.
   The Library is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.
   The Library is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   General Public License for more details.
   The GNU General Public License is often shipped with GNU software, and
   is generally kept in a file called COPYING or LICENSE.  If you do not
   have a copy of the license, write to the Free Software Foundation,
   59 Temple Place, Suite 330, Boston, MA 02111 USA. */

/* Borrowed only those functions from the readline library required for
   the history function implementation of roshell*/

#include <stdlib.h>
#include <string.h>

typedef void *histdata_t;

/* The structure used to store a history entry. */
typedef struct _hist_entry {
  char *line;
} HIST_ENTRY;

/* A structure used to pass the current state of the history stuff around. */
typedef struct _hist_state {
  HIST_ENTRY **entries; /* Pointer to the entries themselves. */
  int offset;           /* The location pointer within this array. */
  int length;           /* Number of elements within this array. */
  int size;             /* Number of slots allocated to this array. */
  int flags;
} HISTORY_STATE;

#define HS_STIFLED 0x01

#define DEFAULT_HISTORY_GROW_SIZE 50

/* Begin a session in which the history functions might be used.  This
   just initializes the interactive variables. */
void using_history(void);

/* Free the history entry H and return any application-specific data
   associated with it. */
void free_history_entry(HIST_ENTRY *);

/* Return the history entry which is logically at OFFSET in the history
   array.  OFFSET is relative to history_base. */
HIST_ENTRY *history_get(int);

/* Place STRING at the end of the history list.
   The associated data field (if any) is set to NULL. */
void add_history(const char *);

/* Clear the history list and start over. */
void clear_history(void);

/* Back up history_offset to the previous history entry, and return
   a pointer to that entry.  If there is no previous entry, return
   a NULL pointer. */
HIST_ENTRY *previous_history(void);

/* Move history_offset forward to the next item in the input_history,
   and return the a pointer to that entry.  If there is no next entry,
   return a NULL pointer. */
HIST_ENTRY *next_history(void);

/* Return a NULL terminated array of HIST_ENTRY which is the current input
   history.  Element 0 of this list is the beginning of time.  If there
   is no history, return NULL. */
HIST_ENTRY **history_list(void);
