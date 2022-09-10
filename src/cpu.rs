use crate::{addressing_mode::AddressingMode, opcode::{OpCode, OPCODES_MAP}};

const PROGRAM_START: u16 = 0x8000;
const RESET_START: u16 = 0xFFFC;

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: u8,
    pub program_counter: u16,
    memory: [u8; 0xFFFF],
}

impl CPU {
    pub fn new() -> Self {
        CPU {
            register_a: 0,
            register_x: 0,
            register_y: 0,
            status: 0,
            program_counter: 0,
            memory: [0; 0xFFFF],
        }
    }

    // Reads a value from a specific addres of memory
    fn mem_read(&self, addr: u16) -> u8 {
        return self.memory[addr as usize];
    }

    // Writes a value into a specific addres of memory
    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }

    // Reads two bytes from memory
    fn mem_read_u16(&mut self, pos: u16) -> u16 {
        // NES is little endian so we need to read them,
        // Then assemble them in the opposite order.
        let low = self.mem_read(pos) as u16;
        let high = self.mem_read(pos + 1) as u16;
        return (high << 8) | (low as u16);
    }

    // Writes two bytes to memory
    fn mem_write_u16(&mut self, pos: u16, data: u16) {
        // NES is little endian so we need to mask the data,
        // then write them in the opposite order
        let low = (data & 0xFF) as u8;
        let high = (data >> 8) as u8;
        self.mem_write(pos, low);
        self.mem_write(pos + 1, high);
    }

    fn get_operand_address(&mut self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,
            AddressingMode::ZeroPage => self.mem_read(self.program_counter) as u16,
            AddressingMode::Absolute => self.mem_read_u16(self.program_counter),

            AddressingMode::ZeroPage_X => {
                let position = self.mem_read(self.program_counter);
                let address = position.wrapping_add(self.register_x) as u16;
                return address;
            }
            AddressingMode::ZeroPage_Y => {
                let position = self.mem_read(self.program_counter);
                let address = position.wrapping_add(self.register_y) as u16;
                return address;
            }

            AddressingMode::Absolute_X => {
                let base = self.mem_read_u16(self.program_counter);
                let address = base.wrapping_add(self.register_x as u16);
                return address;
            }
            AddressingMode::Absolute_Y => {
                let base = self.mem_read_u16(self.program_counter);
                let address = base.wrapping_add(self.register_y as u16);
                return address;
            }

            AddressingMode::Indirect_X => {
                let base = self.mem_read(self.program_counter);

                let pointer: u8 = (base as u8).wrapping_add(self.register_x);
                let low = self.mem_read(pointer as u16);
                let high = self.mem_read(pointer.wrapping_add(1) as u16);
                let deref_base = (high as u16) << 8 | (low as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                return deref;
            }
            AddressingMode::Indirect_Y => {
                let base = self.mem_read(self.program_counter);

                let low = self.mem_read(base as u16);
                let high = self.mem_read((base as u8).wrapping_add(1) as u16);
                let deref_base = (high as u16) << 8 | (low as u16);
                let deref = deref_base.wrapping_add(self.register_y as u16);
                return deref;
            }

            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode);
            }
        }
    }

    fn run_instruction(&mut self, opcode: &OpCode) -> bool {
        match opcode.opcode {
            // LDA
            0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                self.lda(&opcode.mode);
            }
            // STA
            0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                self.sta(&opcode.mode);
            }
            // TAX
            0xAA => self.tax(),
            // INX
            0xE8 => self.inx(),
            // BRK
            0x00 => return false,
            _ => todo!(
                "Opcode at address {:#01x} not implemented: {:#01x}",
                self.program_counter,
                opcode.opcode
            ),
        }
        return true;
    }

    pub fn reset(&mut self) {
        self.register_a = 0;
        self.register_x = 0;
        self.register_y = 0;
        self.status = 0;

        self.program_counter = self.mem_read_u16(RESET_START);
    }

    // Load the program rom into memory
    pub fn load(&mut self, program: Vec<u8>) {
        // The program starts at 0x8000 and runs to 0xFFFF
        self.memory[PROGRAM_START as usize..(PROGRAM_START as usize + program.len())]
            .copy_from_slice(&program[..]);
        self.mem_write_u16(RESET_START, PROGRAM_START);
    }

    // Loads the rom into memory and runs the code
    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    pub fn run(&mut self) {
        loop {
            let code = self.mem_read(self.program_counter); 
            let opcode = OPCODES_MAP
                .get(&code)
                .expect(&format!("OpCode {:#01x} not found in map", code));
            self.program_counter += 1;
            let pc_state = self.program_counter;
            let continue_run = self.run_instruction(opcode);

            if pc_state == self.program_counter {
                self.program_counter += (opcode.bytes - 1) as u16;
            }

            if !continue_run {
                return;
            }
        }
    }

    // Set register a opcode
    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_a = value;
        self.update_zero_and_negative_flags(self.register_a);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.mem_write(address, self.register_a);
    }

    // Copy a to x opcode
    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.update_zero_and_negative_flags(self.register_x);
    }

    // Increment x register opcode
    fn inx(&mut self) {
        // Rust panicks on owerflow
        // NES does not panick,
        // so we need to handle the overflow manually
        if self.register_x == u8::MAX {
            self.register_x = 0;
        } else {
            self.register_x += 1;
        }
        self.update_zero_and_negative_flags(self.register_x);
    }

    fn update_zero_and_negative_flags(&mut self, result: u8) {
        if result == 0 {
            // Zero flag
            self.status = self.status | 0b0000_0010; // On
        } else {
            self.status = self.status & 0b1111_1101; // Off
        }

        if result & 0b1000_0000 != 0 {
            // Negative flag
            self.status = self.status | 0b1000_0000; // On
        } else {
            self.status = self.status & 0b0111_1111; // Off
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_lda_immidiate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x05, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b00);
        assert!(cpu.status & 0b1000_0000 == 0);
    }

    #[test]
    fn test_lda_with_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x00, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);

        cpu.load_and_run(vec![0xA5, 0x10, 0x00]);

        assert_eq!(cpu.register_a, 0x55);
    }

    #[test]
    fn test_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0x0A, 0xAA, 0x00]);
        assert_eq!(cpu.register_x, 10);
    }

    #[test]
    fn test_inx_x_does_increment() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xE8, 0x00]);
        assert_eq!(cpu.register_x, 1);
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0xC0, 0xAA, 0xE8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0xFF, 0xAA, 0xE8, 0xE8, 0x00]);

        assert_eq!(cpu.register_x, 1)
    }
}