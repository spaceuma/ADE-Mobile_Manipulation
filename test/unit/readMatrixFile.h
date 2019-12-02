#ifndef __READ_MATRIX_FILE__
#define __READ_MATRIX_FILE__

void readMatrixFile(std::string map_file,
                    double res,
                    double &n_row,
                    double &n_col,
                    std::vector<double> &vector_elevationData);
#endif
