public class Cell{
    public int row, col;
    public Cell(int row, int col) {
        this.row = row;
        this.col = col;
    }
    public bool Equals(Cell that) {
        if (this.row == that.row && this.col == that.col) {
            return true;
        } else {
            return false;
        }
    }
}