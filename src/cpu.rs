use crate::{
    addressing_mode::AddressingMode,
    opcode::{OpCode, OPCODES_MAP},
};

const PROGRAM_START: u16 = 0x8000;
const RESET_POINTER: u16 = 0xFFFC;
const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xFD;

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

enum Register {
    A,
    X,
    Y,
    Memory,
}

pub struct CPU {
    pub register_a: u8,
    pub register_x: u8,
    pub register_y: u8,
    pub status: u8,
    pub program_counter: u16,
    pub stack_pointer: u8,
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

    fn get_bit_at_n_u8(&self, value: u8, n: u8) -> Result<bool, &'static str> {
        if n >= 8 {
            return Err("n is out of bounds. It is only valid when n < 8");
        }

        let bit: bool = match n {
            0 => self.mask(value, CpuFlags::CARRY, LogicalOperator::AND, false) != 0,
            1 => self.mask(value, CpuFlags::ZERO, LogicalOperator::AND, false) != 0,
            2 => {
                self.mask(
                    value,
                    CpuFlags::INTERRUPT_DISABLE,
                    LogicalOperator::AND,
                    false,
                ) != 0
            }
            3 => self.mask(value, CpuFlags::DECIMAL_MODE, LogicalOperator::AND, false) != 0,
            4 => self.mask(value, CpuFlags::BREAK, LogicalOperator::AND, false) != 0,
            5 => self.mask(value, CpuFlags::BREAK2, LogicalOperator::AND, false) != 0,
            6 => self.mask(value, CpuFlags::OVERFLOW, LogicalOperator::AND, false) != 0,
            7 => self.mask(value, CpuFlags::NEGATIVE, LogicalOperator::AND, false) != 0,
            _ => panic!("How did you get here"),
        };

        return Ok(bit);
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
    fn is_interupt_disable(&self, value: u8) -> bool;
    fn set_interupt_disable(&mut self, value: bool);
    fn set_interupt_disable_from_value(&mut self, value: u8);
    // DECIMAL MODE
    fn is_decimal(&self, value: u8) -> bool;
    fn set_decimal(&mut self, value: bool);
    fn set_decimal_from_value(&mut self, value: u8);
    // BREAK
    fn is_break(&self, value: u8) -> bool;
    fn set_break(&mut self, value: bool);
    fn set_break_from_value(&mut self, value: u8);
    // BREAK2
    fn is_break2(&self, value: u8) -> bool;
    fn set_break2(&mut self, value: bool);
    fn set_break2_from_value(&mut self, value: u8);
    // OVERFLOW
    fn is_overflow(&self, value: u16) -> bool;
    fn set_overflow(&mut self, value: bool);
    fn set_overflow_from_value(&mut self, value: u8, result: u8);
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
    fn is_interupt_disable(&self, _value: u8) -> bool {
        todo!();
    }

    fn set_interupt_disable(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::INTERRUPT_DISABLE);
    }

    fn set_interupt_disable_from_value(&mut self, _value: u8) {
        todo!();
    }

    // DECIMAL MODE
    fn is_decimal(&self, _value: u8) -> bool {
        todo!();
    }

    fn set_decimal(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::DECIMAL_MODE);
    }

    fn set_decimal_from_value(&mut self, _value: u8) {
        todo!();
    }

    // BREAK
    fn is_break(&self, _value: u8) -> bool {
        todo!();
    }

    fn set_break(&mut self, _value: bool) {
        todo!();
    }

    fn set_break_from_value(&mut self, _value: u8) {
        todo!();
    }

    // BREAK2
    fn is_break2(&self, _value: u8) -> bool {
        todo!();
    }

    fn set_break2(&mut self, _value: bool) {
        todo!();
    }

    fn set_break2_from_value(&mut self, _value: u8) {
        todo!();
    }

    // OVERFLOW
    fn is_overflow(&self, value: u16) -> bool {
        return value > u8::MAX as u16;
    }

    fn set_overflow(&mut self, value: bool) {
        self.set_flag(value, CpuFlags::OVERFLOW);
    }

    fn set_overflow_from_value(&mut self, value: u8, result: u8) {
        let m = self.register_a;
        let n = value;
        let c = result;

        let left_inner =
            !(self.get_bit_at_n_u8(m, 7).unwrap() | self.get_bit_at_n_u8(n, 7).unwrap());
        let left = left_inner & self.get_bit_at_n_u8(c, 6).unwrap();

        let right_inner =
            !(self.get_bit_at_n_u8(m, 7).unwrap() | self.get_bit_at_n_u8(n, 7).unwrap());
        let right = !(right_inner | self.get_bit_at_n_u8(c, 6).unwrap());

        let formula = !(!(left | right));
        self.set_overflow(formula);
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
        self.set_negative_from_value(result);
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
            stack_pointer: STACK_RESET,
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

            AddressingMode::Indirect => {
                let base = self.mem_read_u16(self.program_counter);

                if base & 0x00FF == 0x00FF {
                    let low = self.mem_read(base);
                    let high = self.mem_read(base);
                    return (high as u16) << 8 | (low as u16);
                } else {
                    return self.mem_read_u16(base);
                };
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
        println!(
            "OP: {}, PC: {:#01x}, A: {:#02x}, X: {:#02x}, Y: {:#02x}",
            opcode.name, self.program_counter-1, self.register_a, self.register_x, self.register_y
        );
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
            0x24 | 0x2C => self.bit(&opcode.mode),
            // BMI
            0x30 => self.bmi(),
            // BNE
            0xD0 => self.bne(),
            // BPL
            0x10 => self.bpl(),
            // BRK
            0x00 => return false,
            // BVC
            0x50 => self.bvc(),
            // BVS
            0x70 => self.bvs(),
            // CLC
            0x18 => self.clc(),
            // CLD
            0xD8 => self.cld(),
            // CLI
            0x58 => self.cli(),
            // CLV
            0xB8 => self.clv(),
            // CMP
            0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                self.cmp(&opcode.mode);
            }
            // CPX
            0xE0 | 0xE4 | 0xEC => {
                self.cpx(&opcode.mode);
            }
            // CPY
            0xC0 | 0xC4 | 0xCC => {
                self.cpy(&opcode.mode);
            }
            // DEC
            0xC6 | 0xD6 | 0xCE | 0xDE => {
                self.dec(&opcode.mode);
            }
            // DEX
            0xCA => self.dex(),
            // DEY
            0x88 => self.dey(),
            // EOR
            0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                self.eor(&opcode.mode);
            }
            // INC
            0xE6 | 0xF6 | 0xEE | 0xFE => {
                self.inc(&opcode.mode);
            }
            // INX
            0xE8 => self.inx(),
            // INY
            0xC8 => self.iny(),
            // JMP
            0x4C | 0x6C => {
                self.jmp(&opcode.mode);
            }
            // JSR
            0x20 => self.jsr(),
            // LDA
            0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                self.lda(&opcode.mode);
            }
            // LDX
            0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                self.ldx(&opcode.mode);
            }
            // LDY
            0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                self.ldy(&opcode.mode);
            }
            // LSR
            0x4A | 0x46 | 0x56 | 0x4E | 0x5E => {
                self.lsr(&opcode.mode);
            }
            // NOP
            0xEA => {}
            // EOR
            0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                self.ora(&opcode.mode);
            }
            // PHA
            0x48 => self.pha(),
            // PHP
            0x08 => self.php(),
            // PLA
            0x68 => self.pla(),
            // PLP
            0x28 => self.plp(),
            // ROL
            0x2A | 0x26 | 0x36 | 0x2E | 0x3E => {
                self.rol(&opcode.mode);
            }
            // ROR
            0x6A | 0x66 | 0x76 | 0x6E | 0x7E => {
                self.ror(&opcode.mode);
            }
            // RTI
            0x40 => self.rti(),
            0x60 => self.rts(),
            // STA
            0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                self.sta(&opcode.mode);
            }
            // TAX
            0xAA => self.tax(),
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
        self.status = 0b100100;
        self.stack_pointer = STACK_RESET;

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

    fn stack_pop(&mut self) -> u8 {
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        return self.mem_read((STACK as u16) + self.stack_pointer as u16);
    }

    fn stack_push(&mut self, value: u8) {
        self.mem_write((STACK as u16) + self.stack_pointer as u16, value);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn stack_pop_u16(&mut self) -> u16 {
        let low = self.stack_pop() as u16;
        let high = self.stack_pop() as u16;
        return high << 8 | low;
    }

    fn stack_push_u16(&mut self, value: u16) {
        let high = (value >> 8) as u8;
        let low = (value & 0xFF) as u8;
        self.stack_push(high);
        self.stack_push(low);
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

    fn comparison(&mut self, register: Register, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        let reg = match register {
            Register::A => self.register_a,
            Register::X => self.register_x,
            Register::Y => self.register_y,
            Register::Memory => panic!("NO!"),
        };
        self.set_carry(reg >= value);
        self.set_zero(reg == value);
        self.set_negative((reg.wrapping_sub(value)) & 0b1000_0000 != 0);
    }

    fn increment(&mut self, register: Register, mode: &AddressingMode) {
        match register {
            Register::A => self.register_a += 1,
            Register::X => self.register_x += 1,
            Register::Y => self.register_y += 1,
            Register::Memory => {
                let address = self.get_operand_address(mode);
                let value = self.mem_read(address);

                let result = value.wrapping_add(1);

                self.mem_write(address, result);

                self.set_zero(result == 0);
                self.set_negative(result & 0b1000_0000 != 0);
                return;
            }
        };

        let result = match register {
            Register::A => self.register_a,
            Register::X => self.register_x,
            Register::Y => self.register_y,
            Register::Memory => panic!("HOW DID YOU GET HERE!?!"),
        };

        self.set_zero(result == 0);
        self.set_negative(result & 0b1000_0000 != 0);
    }

    fn decrement(&mut self, register: Register, mode: &AddressingMode) {
        match register {
            Register::A => self.register_a -= 1,
            Register::X => self.register_x -= 1,
            Register::Y => self.register_y -= 1,
            Register::Memory => {
                let address = self.get_operand_address(mode);
                let value = self.mem_read(address);

                let result = value.wrapping_sub(1);

                self.mem_write(address, result);

                self.set_zero(result == 0);
                self.set_negative(result & 0b1000_0000 != 0);
                return;
            }
        };

        let result = match register {
            Register::A => self.register_a,
            Register::X => self.register_x,
            Register::Y => self.register_y,
            Register::Memory => panic!("HOW DID YOU GET HERE!?!"),
        };

        self.set_zero(result == 0);
        self.set_negative(result & 0b1000_0000 != 0);
    }

    // OPERATIONS
    // Add with carry
    fn adc(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        let (sum, overflowed) = self.register_a.overflowing_add(value);

        self.set_carry(overflowed);
        self.set_overflow_from_value(value, sum);
        self.register_a = sum;
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
    fn bit(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        // Set zero if A and value == 0
        self.set_zero(self.register_a & value == 0);

        self.set_overflow(value & 0b0100_0000 != 0);
        self.set_negative(value & 0b1000_0000 != 0);
    }

    // Branch if minus
    fn bmi(&mut self) {
        self.branch(
            !self.get_flag(CpuFlags::NEGATIVE),
            &AddressingMode::Immediate,
        );
    }

    // Branch if not equal
    fn bne(&mut self) {
        self.branch(self.get_flag(CpuFlags::ZERO), &AddressingMode::Immediate);
    }

    // Branch if positive
    fn bpl(&mut self) {
        self.branch(
            self.get_flag(CpuFlags::NEGATIVE),
            &AddressingMode::Immediate,
        );
    }

    // Branch if overflow clear
    fn bvc(&mut self) {
        self.branch(
            self.get_flag(CpuFlags::OVERFLOW),
            &AddressingMode::Immediate,
        );
    }

    // Branch if overflow set
    fn bvs(&mut self) {
        self.branch(
            !self.get_flag(CpuFlags::OVERFLOW),
            &AddressingMode::Immediate,
        );
    }

    // Clear carry flag
    fn clc(&mut self) {
        self.set_carry(false);
    }

    // Clear decimal mode
    fn cld(&mut self) {
        self.set_decimal(false);
    }

    // Clear interrupt disable
    fn cli(&mut self) {
        self.set_interupt_disable(false);
    }

    // Clear overflow flag
    fn clv(&mut self) {
        self.set_overflow(false);
    }

    // Compare
    fn cmp(&mut self, mode: &AddressingMode) {
        self.comparison(Register::A, mode);
    }

    // Compare x register
    fn cpx(&mut self, mode: &AddressingMode) {
        self.comparison(Register::X, mode);
    }

    // Compare y register
    fn cpy(&mut self, mode: &AddressingMode) {
        self.comparison(Register::Y, mode);
    }

    // Decrement memory
    fn dec(&mut self, mode: &AddressingMode) {
        self.decrement(Register::Memory, mode);
    }

    // Decrement x register
    fn dex(&mut self) {
        self.decrement(Register::X, &AddressingMode::NoneAddressing);
    }

    // Decrement y register
    fn dey(&mut self) {
        self.decrement(Register::Y, &AddressingMode::NoneAddressing);
    }

    // Exclusive or
    fn eor(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_a ^= value;

        self.set_zero(self.register_a == 0);
        self.set_negative(self.register_a & 0b1000_0000 != 0);
    }

    // Increment memory
    fn inc(&mut self, mode: &AddressingMode) {
        self.increment(Register::Memory, mode);
    }

    // Increment x register
    fn inx(&mut self) {
        self.register_x = self.register_x.wrapping_add(1);

        self.set_zero_and_negative(self.register_x);
    }

    // Increment y register
    fn iny(&mut self) {
        self.register_y = self.register_y.wrapping_add(1);

        self.set_zero(self.register_y == 0);
        self.set_negative(self.register_y & 0b1000_0000 != 0);
    }

    // Jump
    fn jmp(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read_u16(address);

        self.program_counter = value;
    }

    // Jump to subroutine
    fn jsr(&mut self) {
        let address = self.get_operand_address(&AddressingMode::Absolute);
        let value = self.mem_read_u16(address);

        self.stack_push_u16(self.program_counter + 2);
        self.program_counter = value;
    }

    // Set register a
    fn lda(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_a = value;
        self.set_zero_and_negative(self.register_a);
    }

    // Set register x
    fn ldx(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_x = value;

        self.set_zero(value == 0);
        self.set_negative(value & 0b1000_0000 != 0);
    }

    // Set register y
    fn ldy(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_y = value;

        self.set_zero(value == 0);
        self.set_negative(value & 0b1000_0000 != 0);
    }

    // Logical shift right
    fn lsr(&mut self, mode: &AddressingMode) {
        let mut address: u16 = 0;

        let mut value: u8 = if mode == &AddressingMode::NoneAddressing {
            self.register_a
        } else {
            address = self.get_operand_address(mode);
            self.mem_read(address)
        };

        self.set_carry(value & 0b0000_0001 != 0);

        value >>= 1;

        if mode == &AddressingMode::NoneAddressing {
            self.register_a = value;
        } else {
            self.mem_write(address, value);
        }

        self.set_zero(value == 0);
        self.set_negative(value & 0b1000_0000 != 0);
    }

    // Logical inclusive or
    fn ora(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.mem_read(address);

        self.register_a |= value;

        self.set_zero(self.register_a == 0);
        self.set_negative(self.register_a & 0b1000_0000 != 0);
    }

    // Push accumulator
    fn pha(&mut self) {
        self.stack_push(self.register_a);
    }

    // Push processor status
    fn php(&mut self) {
        self.stack_push(self.status);
    }

    // Pull accumulator
    fn pla(&mut self) {
        self.register_a = self.stack_pop();

        self.set_zero(self.register_a == 0);
        self.set_negative(self.register_a & 0b1000_0000 != 0);
    }

    // Pull processor status
    fn plp(&mut self) {
        self.status = self.stack_pop();
    }

    // Rotate left
    fn rol(&mut self, mode: &AddressingMode) {
        let mut address = 0;
        let value = if mode == &AddressingMode::NoneAddressing {
            self.register_a
        } else {
            address = self.get_operand_address(mode);
            self.mem_read(address)
        };

        let result = value.rotate_left(1);

        if mode == &AddressingMode::NoneAddressing {
            self.register_a = result;
        } else {
            self.mem_write(address, result);
        }

        self.set_carry(value & 0b1000_0000 != 0);
        self.set_zero(result == 0);
        self.set_negative(result & 0b1000_0000 != 0);
    }

    // Rotate right
    fn ror(&mut self, mode: &AddressingMode) {
        let mut address = 0;
        let value = if mode == &AddressingMode::NoneAddressing {
            self.register_a
        } else {
            address = self.get_operand_address(mode);
            self.mem_read(address)
        };

        let result = value.rotate_right(1);

        if mode == &AddressingMode::NoneAddressing {
            self.register_a = result;
        } else {
            self.mem_write(address, result);
        }

        self.set_carry(value & 0b0000_0001 != 0);
        self.set_zero(result == 0);
        self.set_negative(result & 0b1000_0000 != 0);
    }

    // Return from interrupt
    fn rti(&mut self) {
        self.status = self.stack_pop();
        self.program_counter = self.stack_pop_u16();
    }

    // Return from subroutine
    fn rts(&mut self) {
        self.program_counter = self.stack_pop_u16();
    }

    // Store accumulator
    fn sta(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        self.mem_write(address, self.register_a);
    }

    // Copy a to x opcode
    fn tax(&mut self) {
        self.register_x = self.register_a;
        self.set_zero_and_negative(self.register_x);
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::vec;

    // HELPERS
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

    // TEX
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

    //  ADC
    #[test]
    fn test_adc_adds() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x10, 0x69, 0x15, 0x00], None);

        assert_eq!(cpu.register_a, 0x10 + 0x15);

        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
        assert!(!cpu.get_flag(CpuFlags::OVERFLOW));
    }

    #[test]
    fn test_adc_adds_and_overflows() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x69, 0x10, 0x00], None);

        // Sum correct
        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10));
        // Overflow occurred
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // AND
    #[test]
    fn test_op_and() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x5A, 0x29, 0x33], None);

        assert_eq!(cpu.register_a, 0x5A & 0x33);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // ASL
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

    // BCC
    #[test]
    fn test_bcc_does_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x10, 0x90, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x0A), (0x80F2, 0x00)]),
        );

        assert_eq!(cpu.register_a, 0x10 << 1);
    }

    #[test]
    fn test_bcc_does_not_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0xFF, 0x69, 0x10, 0x90, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x0A), (0x80F2, 0x00)]),
        );

        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10));
    }

    // BCS
    #[test]
    fn test_bcs_does_not_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x10, 0xB0, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x0A), (0x80F2, 0x00)]),
        );

        assert_eq!(cpu.register_a, 0x10);
    }

    #[test]
    fn test_bcs_does_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0xFF, 0x69, 0x10, 0xB0, 0xEE, 0x00],
            Some(vec![(0x80F3, 0x0A), (0x80F4, 0x00)]),
        );

        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10) << 1);
    }

    // BEQ
    #[test]
    fn test_beq_does_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x00, 0xF0, 0xEE, 0x00],
            Some(vec![(0x80F1, 0xA9), (0x80F2, 0x69), (0x80F3, 0x00)]),
        );

        assert_eq!(cpu.register_a, 0x69);
    }

    #[test]
    fn test_beq_does_not_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x10, 0x69, 0x10, 0xF0, 0xEE, 0x00],
            Some(vec![(0x80F3, 0x0A), (0x80F4, 0x00)]),
        );

        assert_eq!(cpu.register_a, (0x10 as u8).wrapping_add(0x10));
    }

    // BIT
    #[test]
    fn test_bit_test_is_zero() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0b1010_1010, 0x24, 0xEE, 0x00],
            Some(vec![(0xEE, 0b0101_0101)]),
        );

        assert!(cpu.get_flag(CpuFlags::ZERO));

        assert!(cpu.get_flag(CpuFlags::OVERFLOW));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_bit_test_is_not_zero() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0b1010_1010, 0x24, 0xEE, 0x00],
            Some(vec![(0xEE, 0b0101_1010)]),
        );

        assert!(!cpu.get_flag(CpuFlags::ZERO));

        assert!(cpu.get_flag(CpuFlags::OVERFLOW));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // BMI
    #[test]
    fn test_bmi_does_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0xFF, 0x30, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x69), (0x80F2, 0x10), (0x80F3, 0x00)]),
        );

        assert_eq!(cpu.register_a, (0xFF as u8).wrapping_add(0x10));
    }

    #[test]
    fn test_bmi_does_not_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x0F, 0x30, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x69), (0x80F2, 0x10), (0x80F3, 0x00)]),
        );

        assert_eq!(cpu.register_a, 0x0F);
    }

    // BNE
    #[test]
    fn test_bne_does_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x01, 0xD0, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x69), (0x80F2, 0x69)]),
        );

        assert_eq!(cpu.register_a, 0x01 + 0x69);
    }

    #[test]
    fn test_bne_does_not_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x00, 0xD0, 0xEE, 0x69, 0x15, 0x00],
            Some(vec![(0x80F1, 0x69), (0x80F2, 0x69)]),
        );

        assert_eq!(cpu.register_a, 0x15);
    }

    // BPL
    #[test]
    fn test_bpl_does_not_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0xFF, 0x10, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x69), (0x80F2, 0x10), (0x80F3, 0x00)]),
        );

        assert_eq!(cpu.register_a, 0xFF);
    }

    #[test]
    fn test_bpl_does_branch() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x0F, 0x10, 0xEE, 0x00],
            Some(vec![(0x80F1, 0x69), (0x80F2, 0x10), (0x80F3, 0x00)]),
        );

        assert_eq!(cpu.register_a, 0x0F + 0x10);
    }

    // CMP
    #[test]
    fn test_cmp_a_and_value_equal() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0x69], None);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_cmp_a_bigger_than_value() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0x13], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_cmp_a_smaller_than_value() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0xFF], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_cmp_neg_flag_set() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0xC9, 0x0F], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // CPX
    // TODO: IMPLEMENT AFTER LDX
    //#[test]
    fn _test_cpx_x_and_value_equal() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0x69], None);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // TODO: IMPLEMENT AFTER LDX
    // #[test]
    fn _test_cpx_x_bigger_than_value() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0x13], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // TODO: IMPLEMENT AFTER LDX
    //#[test]
    fn _test_cpx_x_smaller_than_value() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0xFF], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // TODO: IMPLEMENT AFTER LDX
    //#[test]
    fn _test_cpx_neg_flag_set() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0xC9, 0x0F], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // CPY
    // TODO: IMPLEMENT AFTER LDX
    //#[test]
    fn _test_cpy_y_and_value_equal() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0x69], None);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // TODO: IMPLEMENT AFTER LDX
    // #[test]
    fn _test_cpy_y_bigger_than_value() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0x13], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // TODO: IMPLEMENT AFTER LDX
    //#[test]
    fn _test_cpy_y_smaller_than_value() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0xC9, 0xFF], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // TODO: IMPLEMENT AFTER LDX
    //#[test]
    fn _test_cpy_neg_flag_set() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0xC9, 0x0F], None);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // DEC
    #[test]
    fn test_dec_zp_decrements() {
        let cpu = get_cpu_with_program(vec![0xC6, 0xEE], Some(vec![(0xEE, 0xFF)]));

        assert_eq!(cpu.mem_read(0xEE), 0xFE);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_dec_zp_result_zero() {
        let cpu = get_cpu_with_program(vec![0xC6, 0xEE], Some(vec![(0xEE, 0x01)]));

        assert_eq!(cpu.mem_read(0xEE), 0x00);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // DEX
    #[test]
    fn test_dex_zp_decrements() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0xAA, 0xCA], None);

        assert_eq!(cpu.register_x, 0xFE);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_dex_zp_result_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x01, 0xAA, 0xCA], None);

        assert_eq!(cpu.register_x, 0x00);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // DEY
    // TODO: IMPLEMENT WHEN Y REGISTER HAS A FUNCTION
    //#[test]
    fn _test_dey_zp_decrements() {}

    // TODO: IMPLEMENT WHEN Y REGISTER HAS A FUNCTION
    //#[test]
    fn _test_dey_zp_result_zero() {}

    // EOR
    #[test]
    fn test_eor_should_be_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0b1010_0101, 0x49, 0b1010_0101], None);

        assert_eq!(cpu.register_a, 0);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_eor_should_be_negative() {
        let cpu = get_cpu_with_program(vec![0xA9, 0b1010_0101, 0x49, 0b0101_1010], None);

        assert_eq!(cpu.register_a, 0xFF);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // INC
    #[test]
    fn test_inc_is_zero() {
        let cpu = get_cpu_with_program(vec![0xE6, 0xEE], Some(vec![(0xEE, 0xFF)]));

        assert_eq!(cpu.mem_read(0xEE), 0x00);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_inc_is_negative() {
        let cpu = get_cpu_with_program(vec![0xE6, 0xEE], Some(vec![(0xEE, 0xFE)]));

        assert_eq!(cpu.mem_read(0xEE), 0xFF);

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // INX
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

    // INY
    // TODO: TEST THIS

    // JMP
    #[test]
    fn test_jmp_to_address() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x69, 0x4C, 0xEE, 0x69],
            Some(vec![
                (0x69EE, 0xA9),
                (0x69EF, 0xF1),
                (0xF1A9, 0xA9),
                (0xF1AA, 0xF1),
            ]),
        );

        assert_eq!(cpu.register_a, 0xF1);
    }

    #[test]
    fn test_jsr_does_jump() {
        let cpu = get_cpu_with_program(
            vec![0xA9, 0x69, 0x20, 0xF1, 0xAA, 0xA9, 0x99],
            Some(vec![
                (0xAAF1, 0xA9),
                (0xAAF2, 0x42),
                (0x42A9, 0xA9),
                (0x42AA, 0x42),
            ]),
        );

        assert_eq!(cpu.register_a, 0x42);
        assert!(cpu.stack_pointer < STACK_RESET);
    }

    // LDA
    #[test]
    fn test_lda_immidiate_load_data() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x05, 0x00], None);

        assert_eq!(cpu.register_a, 0x05);
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

    // LDX
    #[test]
    fn test_ldx_immidiate_load_data() {
        let cpu = get_cpu_with_program(vec![0xA2, 0x05, 0x00], None);

        assert_eq!(cpu.register_x, 0x05);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_ldx_with_zero_flag() {
        let cpu = get_cpu_with_program(vec![0xA2, 0x00, 0x00], None);

        assert_eq!(cpu.register_x, 0x00);
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_ldx_from_memory() {
        let cpu = get_cpu_with_program(vec![0xA6, 0x10, 0x00], Some(vec![(0x10, 0x55)]));

        assert_eq!(cpu.register_x, 0x55);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // LDY
    #[test]
    fn test_ldy_immidiate_load_data() {
        let cpu = get_cpu_with_program(vec![0xA0, 0x05, 0x00], None);

        assert_eq!(cpu.register_y, 0x05);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_ldy_with_zero_flag() {
        let cpu = get_cpu_with_program(vec![0xA0, 0x00, 0x00], None);

        assert_eq!(cpu.register_y, 0x00);
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_ldy_from_memory() {
        let cpu = get_cpu_with_program(vec![0xA4, 0x10, 0x00], Some(vec![(0x10, 0x55)]));

        assert_eq!(cpu.register_y, 0x55);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // LSR
    #[test]
    fn test_lsr_has_carry_and_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x01, 0x4A], None);

        assert_eq!(cpu.register_a, 0x00);
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_lsr_result_negative() {
        let cpu = get_cpu_with_program(vec![0x46, 0xEE], Some(vec![(0xEE, 0xF0)]));

        assert_eq!(cpu.mem_read(0xEE), 0xF0 >> 1);
        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // ORA
    #[test]
    fn test_ora_result_is_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x00, 0x09, 0x00], None);

        assert_eq!(cpu.register_a, 0x00);
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_ora_result_is_negative() {
        let cpu = get_cpu_with_program(vec![0xA9, 0b1010_0101, 0x09, 0b1111_0000], None);

        assert_eq!(cpu.register_a, 0b1111_0101);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // PHA
    #[test]
    fn test_pha_pushes_a() {
        let mut cpu = get_cpu_with_program(vec![0xA9, 0x69, 0x48], None);

        assert_eq!(cpu.stack_pop(), 0x69);
    }

    // PHP
    #[test]
    fn test_php_pushes_status() {
        let mut cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x08, 0xA9, 0x00], None);

        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));

        cpu.status = cpu.stack_pop();

        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // PLA
    #[test]
    fn test_pla_is_zero() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x00, 0x48, 0xA9, 0x69, 0x68], None);

        assert_eq!(cpu.register_a, 0);
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(!cpu.get_flag(CpuFlags::NEGATIVE));
    }

    #[test]
    fn test_pla_is_negative() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x48, 0xA9, 0x69, 0x68], None);

        assert_eq!(cpu.register_a, 0xFF);
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // PLP
    #[test]
    fn test_plp_pulls_status() {
        let cpu = get_cpu_with_program(vec![0xA9, 0xFF, 0x48, 0x28], None);

        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::INTERRUPT_DISABLE));
        assert!(cpu.get_flag(CpuFlags::DECIMAL_MODE));
        assert!(cpu.get_flag(CpuFlags::BREAK));
        assert!(cpu.get_flag(CpuFlags::BREAK2));
        assert!(cpu.get_flag(CpuFlags::OVERFLOW));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // ROL
    #[test]
    fn test_rol_rotates() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0x2A], None);

        assert_eq!(cpu.register_a, 0xD2);
        assert!(!cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // ROR
    #[test]
    fn test_ror_rotates() {
        let cpu = get_cpu_with_program(vec![0xA9, 0x69, 0x6A], None);

        assert_eq!(cpu.register_a, 0xB4);
        assert!(cpu.get_flag(CpuFlags::CARRY));
        assert!(!cpu.get_flag(CpuFlags::ZERO));
        assert!(cpu.get_flag(CpuFlags::NEGATIVE));
    }

    // RTI
    // TODO: TEST THIS

    // RTS
    #[test]
    fn test_rts_here_and_back() {
        let cpu = get_cpu_with_program(
            vec![0xA2, 0x67, 0x20, 0xAA, 0xAA, 0xE8],
            Some(vec![
                (0xAAAA, 0xBB),
                (0xAAAB, 0xBB),
                (0xBBBB, 0xE8),
                (0xBBBC, 0x60),
            ]),
        );

        assert_eq!(cpu.register_x, 0x69);
    }
}
