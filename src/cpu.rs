use crate::{
    addressing_mode::AddressingMode,
    opcode::{OpCode, OPCODES_MAP},
};

const PROGRAM_START: u16 = 0x8000;
const RESET_POINTER: u16 = 0xFFFC;

bitflags! {
    pub struct CpuFlags: u8 {
        const CARRY             = 0b00000001;
        const ZERO              = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL_MODE      = 0b00001000;
        const BREAK             = 0b00010000;
        const BREAK2            = 0b00100000;
        const OVERFLOW          = 0b01000000;
        const NEGATIVE          = 0b10000000;
    }
}

enum LogicalOperator {
    AND,
    OR,
    // XOR,
}

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: u8,
    pub program_counter: u16,
    memory: [u8; 0xFFFF],
}

trait Mem {
    fn mem_read(&self, address: u16) -> u8;
    fn mem_write(&mut self, address: u16, value: u8);

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
}

impl Mem for CPU {
    // Reads a value from a specific addres of memory
    fn mem_read(&self, address: u16) -> u8 {
        return self.memory[address as usize];
    }

    // Writes a value into a specific addres of memory
    fn mem_write(&mut self, address: u16, data: u8) {
        self.memory[address as usize] = data;
    }
}

trait Flags {
    fn mask(&self, value: u8, mask: CpuFlags, operator: LogicalOperator, invert_mask: bool) -> u8 {
        let mask_bits = if invert_mask { !mask.bits } else { mask.bits };
        return match operator {
            LogicalOperator::AND => value & mask_bits,
            LogicalOperator::OR => value | mask_bits,
            // LogicalOperator::XOR => value ^ mask_bits,
        };
    }

    fn set_flag(&mut self, value: bool, flag: CpuFlags);
    fn get_flag(&self, flag: CpuFlags) -> bool;

    // CARRY
    fn is_carry(&self, value: u8) -> bool;
    fn set_carry(&mut self, value: bool);
    fn set_carry_from_value(&mut self, value: u8);

    // ZERO
    fn is_zero(&self, value: u8) -> bool;
    fn set_zero(&mut self, value: bool);
    fn set_zero_from_value(&mut self, value: u8);

    // INTERUPT_DISABLE
    // DECIMAL MODE
    // BREAK
    // BREAK2
    // OVERFLOW
    fn is_overflow(&self, value: u16) -> bool;
    fn set_overflow(&mut self, value: bool);
    fn set_overflow_from_value(&mut self, value: u16);

    // NEGATIVE
    fn is_negative(&self, value: u8) -> bool;
    fn set_negative(&mut self, value: bool);
    fn set_negative_from_value(&mut self, value: u8);

    // HELPERS
    fn set_zero_and_negative(&mut self, result: u8);
}

impl Flags for CPU {
    fn set_flag(&mut self, value: bool, mask: CpuFlags) {
        // println!("SETTING FLAG: {:?} = {}", mask, value);
        let (operator, invert) = match value {
            true => (LogicalOperator::OR, false),
            false => (LogicalOperator::AND, true),
        };
        self.status = self.mask(self.status, mask, operator, invert);
    }

    fn get_flag(&self, flag: CpuFlags) -> bool {
        return self.mask(self.status, flag, LogicalOperator::AND, false) == flag.bits;
    }

    // CARRY
    fn is_carry(&self, value: u8) -> bool {
        return value & 0b1000_0000 != 0;
        // NOTE: I don't think this is right
    }

    fn set_carry(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::CARRY);
    }

    fn set_carry_from_value(&mut self, value: u8) {
        self.set_carry(self.is_carry(value));
    }

    // ZERO
    fn is_zero(&self, value: u8) -> bool {
        return value == 0;
    }

    fn set_zero(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::ZERO);
    }

    fn set_zero_from_value(&mut self, value: u8) {
        self.set_zero(self.is_zero(value));
    }

    // INTERUPT_DISABLE
    // DECIMAL MODE
    // BREAK
    // BREAK2
    // OVERFLOW
    fn is_overflow(&self, value: u16) -> bool {
        return value > u8::MAX as u16;
    }

    fn set_overflow(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::OVERFLOW);
    }

    fn set_overflow_from_value(&mut self, value: u16) {
        self.set_overflow(self.is_overflow(value));
    }

    // NEGATIVE
    fn is_negative(&self, value: u8) -> bool {
        return value & 0b1000_0000 != 0;
    }

    fn set_negative(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::NEGATIVE);
    }

    fn set_negative_from_value(&mut self, value: u8) {
        self.set_negative(self.is_negative(value));
    }

    // HELPERS
    fn set_zero_and_negative(&mut self, result: u8) {
        self.set_zero_from_value(result);
        // if result == 0 {
        //     // Zero flag
        //     // self.status = self.status | CpuFlags::ZERO.bits; // On
        //     self.status = self.mask(self.status, CpuFlags::ZERO, LogicalOperator::OR);
        // // On
        // } else {
        //     // self.status = self.status & !CpuFlags::ZERO.bits; // Off
        //     self.status = self.mask(self.status, !CpuFlags::ZERO, LogicalOperator::AND); // Off
        // }
        self.set_negative_from_value(result);
        // if result & CpuFlags::NEGATIVE.bits != 0 {
        //     // Negative flag
        //     // self.status = self.status | 0b1000_0000; // On
        //     self.status = self.mask(self.status, CpuFlags::NEGATIVE, LogicalOperator::OR);
        // // On
        // } else {
        //     // self.status = self.status & 0b0111_1111; // Off
        //     self.status = self.mask(self.status, !CpuFlags::NEGATIVE, LogicalOperator::AND); // Off
        // }
    }
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
        println!("RAN OP: {}", opcode.name);
        match opcode.opcode {
            // ADC
            0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                self.adc(&opcode.mode);
            }
            // AND
            0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                self.and(&opcode.mode);
            }
            // ASL
            0x0A | 0x06 | 0x16 | 0x0E | 0x1E => {
                self.asl(&opcode.mode);
            }
            // BCC
            0x90 => self.bcc(),
            // BCS
            0xB0 => self.bcs(),
            // BEQ
            0xF0 => self.beq(),
            // BIT
            0x24 | 0x2C => {
                self.bit(&opcode.mode)
            }
            // BMI
            0x30 => self.bmi(),
            // BNE
            0xD0 => self.bne(),
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

        self.program_counter = self.mem_read_u16(RESET_POINTER);
    }

    // Load the program rom into memory
    pub fn load(&mut self, program: Vec<u8>) {
        // The program starts at 0x8000 and runs to 0xFFFF
        self.memory[PROGRAM_START as usize..(PROGRAM_START as usize + program.len())]
            .copy_from_slice(&program[..]);
        self.mem_write_u16(RESET_POINTER, PROGRAM_START);
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

    // HELPERS
    fn branch(&mut self, condition_no_branch: bool, mode: &AddressingMode) {
        if condition_no_branch {
            return;
        }

        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.program_counter += value as u16;
    }

    // OPERATIONS
    // Add with carry
    fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        let (sum, overflowed) = self.register_a.overflowing_add(value);

        self.register_a = sum;
        self.set_carry(overflowed);
        self.set_zero_and_negative(self.register_a);
    }

    // Logical and
    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_a &= value;

        self.set_zero_and_negative(self.register_a);
    }

    // Arithmetic shift left
    fn asl(&mut self, mode: &AddressingMode) {
        let mut value: u8;

        match mode {
            AddressingMode::NoneAddressing => {
                self.set_carry_from_value(self.register_a);
                self.register_a <<= 1;
                value = self.register_a;
            }
            _ => {
                let address = self.get_operand_address(mode);
                value = self.mem_read(address);

                self.set_carry_from_value(value);

                value <<= 1;
                self.mem_write(address, value);
            }
        }
        self.set_zero_and_negative(value);
    }

    // Branch if carry clear
    fn bcc(&mut self) {
        self.branch(self.get_flag(CpuFlags::CARRY), &AddressingMode::Immediate)
    }

    // Branch if carry set
    fn bcs(&mut self) {
        self.branch(!self.get_flag(CpuFlags::CARRY), &AddressingMode::Immediate)
    }

    // Branch if equal
    fn beq(&mut self) {
        self.branch(!self.get_flag(CpuFlags::ZERO), &AddressingMode::Immediate)
    }

    // Bit test
    fn bit(&mut self, mode: &AddressingMode)
    {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        // Set zero if A and value == 0
        self.set_zero(self.register_a & value == 0);

        self.set_overflow(value & 0b0100_0000 != 0);
        self.set_negative(value & 0b1000_0000 != 0);
    }

    // Branch if minus
    fn bmi(&mut self) {
        self.branch(!self.get_flag(CpuFlags::NEGATIVE), &AddressingMode::Immediate);
    }

    // Branch if not equal
    fn bne(&mut self) {
        self.branch(self.get_flag(CpuFlags::ZERO), &AddressingMode::Immediate);
    }

    // Set register a opcode
    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_a = value;
        self.set_zero_and_negative(self.register_a);
    }

    fn sta(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.mem_write(address, self.register_a);
    }

    // Copy a to x opcode
    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.set_zero_and_negative(self.register_x);
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
        self.set_zero_and_negative(self.register_x);
    }
}

#[cfg(test)]
mod test {
    use std::vec;

    use super::*;

    fn get_cpu_with_program(program: Vec<u8>, mem_write: Option<Vec<(u16, u8)>>) -> CPU {
        let mut cpu = CPU::new();
        match mem_write {
            Some(tuples) => {
                for (addr, val) in tuples {
                    cpu.mem_write(addr, val);
                }
            }
            None => {}
        }
        cpu.load_and_run(program);
        return cpu;
    }

    #[test]
    fn test_lda_immidiate_load_data() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x05, 0x00], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_lda_with_zero_flag() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x00, 0x00], None);
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_lda_from_memory() {
        let cpu = get_cpu_with_program(vec![0xA5, 0x10, 0x00], Some(vec![(0x10, 0x55)]));

        assert_eq!(cpu.register_a, 0x55);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_tax_move_a_to_x() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x0A, 0xAA, 0x00], None);

        assert_eq!(cpu.register_x, 10);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_tax_with_zero_flag() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x00, 0xAA], None);

        assert_eq!(cpu.register_x, cpu.register_a);
        assert_eq!(cpu.register_x, 0x00);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_inx_x_does_increment() {
        let cpu = get_cpu_with_program(vec![0xE8, 0x00], None);

        assert_eq!(cpu.register_x, 1);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xA9, 0xC0, 0xAA, 0xE8, 0x00]);

        assert_eq!(cpu.register_x, 0xc1);
    }

    #[test]
    fn test_inx_overflow() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0xAA, 0xE8, 0xE8, 0x00], None);

        assert_eq!(cpu.register_x, 1);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_adc_adds() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x10, 0x69, 0x15, 0x00], None);

        assert_eq!(cpu.register_a, 0x10 + 0x15);

        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
        // TODO: OVERFLOW FLAG
    }

    #[test]
    fn test_adc_adds_and_overflows() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x69, 0x10, 0x00], None);

        // Sum correct
        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10));
        // Overflow occurred
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        // assert!(cpu.get_flag(CpuFlags::OVERFLOW)); // TODO: OVERFLOW FLAG
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_op_and() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x5A, 0x29, 0x33], None);

        assert_eq!(cpu.register_a, 0x5A & 0x33);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_asl_shift_no_accum() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x0F, 0x0A], None);

        assert_eq!(cpu.register_a, 0x0F << 1);
        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_asl_shift_zero_page() {
        let cpu = get_cpu_with_program(vec![0x06, 0xF0, 0x00], Some(vec![(0xF0, 0xF1)]));

        assert_eq!(cpu.mem_read(0xF0), 0xF1 << 1);
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_bcc_does_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x10, 0x90, 0xEE, 0x00], Some(vec![(0x80F1, 0x0A), (0x80F2, 0x00)]));

        assert_eq!(cpu.register_a, 0x10 << 1);
    }

    #[test]
    fn test_bcc_does_not_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x69, 0x10, 0x90, 0xEE, 0x00], Some(vec![(0x80F1, 0x0A), (0x80F2, 0x00)]));

        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10));
    }

    #[test]
    fn test_bcs_does_not_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x10, 0xB0, 0xEE, 0x00], Some(vec![(0x80F1, 0x0A), (0x80F2, 0x00)]));

        assert_eq!(cpu.register_a, 0x10);
    }
    
    #[test]
    fn test_bcs_does_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x69, 0x10, 0xB0, 0xEE, 0x00], Some(vec![(0x80F3, 0x0A), (0x80F4, 0x00)]));
        
        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10) << 1);
    }

    #[test]
    fn test_beq_does_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x00, 0xF0, 0xEE, 0x00], Some(vec![(0x80F1, 0xA9), (0x80F2, 0x69), (0x80F3, 0x00)]));

        assert_eq!(cpu.register_a, 0x69);
    }
    
    #[test]
    fn test_beq_does_not_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x10, 0x69, 0x10, 0xF0, 0xEE, 0x00], Some(vec![(0x80F3, 0x0A), (0x80F4, 0x00)]));
        
        assert_eq!(cpu.register_a, (0x10 as u8).wrapping_add(0x10));
    }

    #[test]
    fn test_bit_test_is_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0b1010_1010, 0x24, 0xEE, 0x00], Some(vec![(0xEE, 0b0101_0101)]));

        assert!(cpu.get_flag(CpuFlags::ZERO));

        assert!(cpu.get_flag(CpuFlags::OVERFLOW));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_bit_test_is_not_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0b1010_1010, 0x24, 0xEE, 0x00], Some(vec![(0xEE, 0b0101_1010)]));

        assert!(!cpu.get_flag(CpuFlags::ZERO));

        assert!(cpu.get_flag(CpuFlags::OVERFLOW));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_bmi_does_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x30, 0xEE, 0x00], Some(vec![(0x80F1, 0x69), (0x80F2, 0x10), (0x80F3, 0x00)]));

        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10));
    }

    #[test]
    fn test_bmi_does_not_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x0F, 0x30, 0xEE, 0x00], Some(vec![(0x80F1, 0x69), (0x80F2, 0x10), (0x80F3, 0x00)]));

        assert_eq!(cpu.register_a, 0x0F);
    }

    #[test]
    fn test_bne_does_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x01, 0xD0, 0xEE, 0x00], Some(vec![(0x80F1, 0x69), (0x80F2, 0x69)]));

        assert_eq!(cpu.register_a, 0x01 + 0x69);
    }

    #[test]
    fn test_bne_does_not_branch() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x00, 0xD0, 0xEE, 0x69, 0x15, 0x00], Some(vec![(0x80F1, 0x69), (0x80F2, 0x69)]));

        assert_eq!(cpu.register_a, 0x15);
    }
}
