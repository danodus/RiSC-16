#include <Vdesign.h>

double sc_time_stamp()
{
    return 0.0;
}

void pulse_clk(Vdesign* top) {
    top->contextp()->timeInc(1);
    top->clk = 1;
    top->eval();

    top->contextp()->timeInc(1);
    top->clk = 0;
    top->eval();
}

int main(int argc, char** argv, char** env) {

    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
    contextp->commandArgs(argc, argv);

    Vdesign* top = new Vdesign{contextp.get(), "TOP"};
    while (!contextp->gotFinish()) {
        pulse_clk(top);
        printf("PC: %x, Display: %x\n", top->pc, top->display);
    }

    top->final();
    delete top;
    return 0;
}