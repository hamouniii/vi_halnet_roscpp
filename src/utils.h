#ifndef Utils_H_
#define Utils_H_

enum Hand{LEFT, RIGHT};

enum Frame{CONFIDENT = 1, UNCERTAIN = 0, UNDEFINDED = -1};

class Point2D{
    public:
        Point2D(){};
        
        Point2D(int r, int c){
            this->row = r;
            this->col = c;
        };

        Point2D operator-(const Point2D& b){
            Point2D ret;
            ret.row = this->row - b.row;
            ret.col = this->col - b.col; 
            return ret;
        }

        Point2D operator+(const Point2D& b){
            Point2D ret;
            ret.row = this->row + b.row;
            ret.col = this->col + b.col; 
            return ret;
        }

        Point2D operator/(const int b){
            Point2D ret;
            ret.row = (int) (this->row / b);
            ret.col = (int) (this->col / b);        
            return ret;
        }

        double norm(){ 
            return sqrt((this->row * this->row) + (this-> col * this->col));    
        }

        Point2D normalize(Point2D in){
            return in/in.norm();
            
        }

        bool is_equal(Point2D in){
            if(this->row == in.row && this->col == in.col)
                return true;
            else
                return false;
        }

        std::string to_string(){
            return "(" + std::to_string(this->row) + ", " + std::to_string(this->col) + ")";
        }

        void setRow(int r){this->row = r;}
        void setCol(int c){this->col = c;}
        int getRow(){return this->row;}
        int getCol(){return this->col;}


    private:
        int row;
        int col;
    
};


#endif