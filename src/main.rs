use cpu::CPU;

pub mod cpu;
pub mod opcode;
mod addressing_mode;

#[macro_use]
extern crate lazy_static;

fn main() {
    let mut cpu = CPU::new();
    cpu.load_and_run(vec![0xA9, 0xFF, 0xAA, 0xE8, 0xE8, 0x00]);
}