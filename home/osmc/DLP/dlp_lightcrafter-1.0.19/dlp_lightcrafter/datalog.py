# datalog.py
#
# logs data to Excel compatible tab-separated column file
#
# Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
#
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
#   Neither the name of Texas Instruments Incorporated nor the names of
#   its contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import time
from time import strftime


class DataLog(object):
    def __init__(self, file_path, file_name, file_extension=".xls", add_time_to_file_name=True):
        """
        @type file_path: str
        @type file_name: str
        @type file_extension: str
        @type add_time_to_file_name: bool

        @param file_path: the path to save the file at
        @param file_name: the name of the data file to save at the specified path
        @param file_extension: the file extension to use, if not specified '.xls' will be used
        @param add_time_to_file_name: True to add the current time to the end of the file name, false to not add it.
                                      If not specified, the default is true. Using this will help keep file names unique.

        Creates a new data log at the specified path, with the specified file name, and file extension.
        Note: if the log already exists, then the new log will be appended to the end of
        the existing log.
        i.e. the second log column headers will begin after the data from the first log.

        For an example of how to use logging, see the main at the bottom of this module
        """
        self.file_path = file_path
        self.file_extension = file_extension
        self.add_time_to_file_name = add_time_to_file_name
        self.time_to_add = "" if not add_time_to_file_name else (
            strftime("__%a__%b_%d_%Y__%H_%M_%S__", (time.localtime(time.time()))))

        self.file_name = file_name + self.time_to_add + self.file_extension
        self.header_file_name = os.path.join(self.file_path, "tmp_header_" + self.file_name)
        self.data_file_name = os.path.join(self.file_path, "tmp_data_" + self.file_name)
        self.current_line = []

        # make the file path if it does not exist
        if not os.path.exists(file_path):
            os.makedirs(file_path)

        # create the files
        header_file = open(self.header_file_name, 'a')
        data_file = open(self.data_file_name, 'a')
        header_file.close()
        data_file.close()

    def add_col(self, col_header, data):
        """
        Adds a column to the log, with the column having the value = data.
        This method should be called to fill in a line item in the log with data.
        If the column header already exists, then that column will be reused in the log.
        If the column header does not exists, it will be added after the last column currently in the log.

        @type col_header: str
        Args:
            data: the data to put in the column that has the specified column header.
            col_header: the data will be placed in the column with the specified column header.
        Returns: None
        """
        if not self.__col_header_exists(col_header):
            self.__add_col_header(col_header)
        index = self.__get_index_of_col_header(col_header)
        if index != -1:
            if len(self.current_line) == 0:
                self.current_line = [""]
            amount_to_extend = (index + 1) - len(self.current_line)
            self.current_line.extend("\t" * amount_to_extend)
            self.current_line[index] = ("\t" if index != 0 else "") + str(data)

    def log(self):
        """
        This method will write a new line in the data log, with column values equal to
        those that have been specified by the add_col_to_log function.
        Returns: None
        """
        data_file = open(self.data_file_name, 'a')
        for data_col in self.current_line:
            data_file.write(data_col)
        data_file.write("\n")
        data_file.close()
        self.current_line = []

    def close(self):
        """
        Closes the log and merges the temporary files into the final data log.
        This method should be called once all data logging is complete for this file.
        Returns: None
        """
        if os.path.isfile(self.header_file_name) and os.path.isfile(self.data_file_name):
            try:
                header_file = open(self.header_file_name, 'r')
                data_file = open(self.data_file_name, 'r')
                log_file = open(os.path.join(self.file_path, self.file_name), 'a')
                for line in header_file:
                    log_file.write(line)
                log_file.write("\n")
                for line in data_file:
                    log_file.write(line)
                header_file.close()
                data_file.close()
                log_file.close()
                os.remove(self.header_file_name)
                os.remove(self.data_file_name)
            except IOError:
                print "Error in Logger#close_logging(). Do you already have the log file open?"
                print "Failed to close data logging."
                raise

    def __add_col_header(self, col_header):
        """
        Internal method, do not call this.
        """
        split_headers = self.__get_col_headers()
        if col_header not in split_headers:
            header_file = open(self.header_file_name, 'a')
            header_file.write(("\t" if len(split_headers) > 0 else "") + col_header)
            header_file.close()

    def __get_col_headers(self):
        """
        Internal method, do not call this.
        """
        # get the current headers in the headers file
        header_file = open(self.header_file_name, 'r')
        headers = header_file.readline()
        header_file.close()
        if "\t" not in headers:
            if headers == "":
                split_headers = []
            else:
                split_headers = [headers]
        else:
            split_headers = headers.split("\t")
        return split_headers

    def __col_header_exists(self, header_name):
        """
        Internal method, do not call this.
        """
        headers = self.__get_col_headers()
        return header_name in headers

    def __get_index_of_col_header(self, header_name):
        """
        Internal method, do not call this.
        """
        headers = self.__get_col_headers()
        if header_name in headers:
            return headers.index(header_name)
        return -1


if __name__ == "__main__":
    # make a new logger object to log at the specified path, with the specified file name
    logger = DataLog("C:\\", "test_log")
    # add some columns with arbitrary data in them
    logger.add_col("col 1", "data for row 1, col 1")
    logger.add_col("col 2", "data for row 1, col 2")
    # write a new line in the log with the columns and data specified above.
    logger.log()
    # add 4 columns to the next line, since the first 2 columns already exists, the same
    # columns will be reused for this line, but with new data.
    logger.add_col("col 1", "data for row 2, col 1")
    logger.add_col("col 2", "data for row 2, col 2")
    logger.add_col("col 3", "data for row 2, col 3")
    logger.add_col("col 4", "data for row 2, col 4")
    # write a new line in the log with the columns and data specified above.
    logger.log()
    # add data for the 3rd line in the log, but only use one of the existing columns with new data
    logger.add_col("col 2", "data for row 3, col 2")
    # write a new line in the log with the columns and data specified above (only one column will have data here).
    logger.log()
    # close logging, this will merge the temporary data files into the final log
    logger.close()
