#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
swu_flash_image_create.py

RSDK_S32DS_A53.elf 파일을 S32R45 Flash 이미지로 변환하는 모듈.

출력 포맷:
- YAML 헤더 (4096바이트 패딩)
- bzip2 압축 데이터
- 4096 바이트 경계 패딩

YAML 헤더 구조:
  model: RADAR_4D
  sw_priority: 1
  compression_type: 1
  file_size: <압축 파일 크기>
  crc_32: <압축 파일 CRC32>
  date: <생성 일시>
"""

import os
import bz2
import zlib
from datetime import datetime
from typing import Optional, Tuple


# 기본 YAML 헤더 템플릿
DEFAULT_YAML_HEADER = """model: RADAR_4D
sw_priority: 1
compression_type: 1
file_size: {file_size}
crc_32: {crc_32_hex}
date: {date}
"""

# 패딩 블록 크기
BLOCK_SIZE = 4096


def calculate_crc32(data: bytes) -> int:
    """CRC32 계산

    Args:
        data: 계산할 바이트 데이터

    Returns:
        CRC32 값 (unsigned 32-bit)
    """
    return zlib.crc32(data) & 0xFFFFFFFF


def compress_bz2(data: bytes, compresslevel: int = 9) -> bytes:
    """bzip2 압축

    Args:
        data: 압축할 바이트 데이터
        compresslevel: 압축 레벨 (1-9)

    Returns:
        압축된 바이트 데이터
    """
    return bz2.compress(data, compresslevel=compresslevel)


def pad_to_block(data: bytes, block_size: int = BLOCK_SIZE, pad_byte: bytes = b' ') -> bytes:
    """데이터를 블록 크기의 배수로 패딩

    Args:
        data: 패딩할 바이트 데이터
        block_size: 블록 크기
        pad_byte: 패딩에 사용할 바이트

    Returns:
        패딩된 바이트 데이터
    """
    current_size = len(data)
    if current_size % block_size == 0:
        return data

    padding_size = block_size - (current_size % block_size)
    return data + (pad_byte * padding_size)


def create_yaml_header(file_size: int, crc_32: int, date_str: Optional[str] = None) -> bytes:
    """YAML 헤더 생성

    Args:
        file_size: 압축 파일 크기
        crc_32: 압축 파일 CRC32
        date_str: 날짜 문자열 (기본: 현재 시간)

    Returns:
        4096바이트로 패딩된 YAML 헤더
    """
    if date_str is None:
        date_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # CRC32를 hex 문자열로 변환 (소문자, 0x 접두사 없음)
    crc_32_hex = f"{crc_32:08x}"

    yaml_content = DEFAULT_YAML_HEADER.format(
        file_size=file_size,
        crc_32_hex=crc_32_hex,
        date=date_str
    )

    yaml_bytes = yaml_content.encode('utf-8')

    # 4096 바이트로 패딩 (공백으로)
    if len(yaml_bytes) > BLOCK_SIZE:
        raise ValueError(f"YAML 헤더가 {BLOCK_SIZE} 바이트를 초과합니다: {len(yaml_bytes)} bytes")

    return pad_to_block(yaml_bytes, BLOCK_SIZE, b' ')


def create_flash_image(
    input_path: str,
    output_path: Optional[str] = None,
    progress_callback: Optional[callable] = None
) -> Tuple[str, dict]:
    """Flash 이미지 생성

    Args:
        input_path: 입력 ELF/바이너리 파일 경로
        output_path: 출력 파일 경로 (기본: RSDK_S32DS_Flash_Image_YYMMDD.bz2)
        progress_callback: 진행 상황 콜백 함수 (percent, message)

    Returns:
        (출력 파일 경로, 정보 딕셔너리)

    정보 딕셔너리:
        - original_size: 원본 파일 크기
        - compressed_size: 압축 파일 크기
        - crc_32: CRC32 값
        - final_size: 최종 출력 파일 크기
        - compression_ratio: 압축률 (%)
    """
    def report_progress(percent: int, message: str):
        if progress_callback:
            progress_callback(percent, message)

    # 1. 입력 파일 읽기
    report_progress(0, "Reading input file...")
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"입력 파일을 찾을 수 없습니다: {input_path}")

    with open(input_path, 'rb') as f:
        original_data = f.read()

    original_size = len(original_data)
    report_progress(10, f"Input file size: {original_size:,} bytes")

    # 2. bzip2 압축
    report_progress(20, "Compressing with bzip2...")
    compressed_data = compress_bz2(original_data, compresslevel=9)
    compressed_size = len(compressed_data)

    compression_ratio = (1 - compressed_size / original_size) * 100 if original_size > 0 else 0
    report_progress(50, f"Compressed: {compressed_size:,} bytes ({compression_ratio:.1f}% reduction)")

    # 3. CRC32 계산
    report_progress(60, "Calculating CRC32...")
    crc_32 = calculate_crc32(compressed_data)
    report_progress(70, f"CRC32: 0x{crc_32:08X}")

    # 4. YAML 헤더 생성
    report_progress(75, "Creating YAML header...")
    date_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    yaml_header = create_yaml_header(compressed_size, crc_32, date_str)

    # 5. 압축 데이터 패딩 (4096 경계)
    report_progress(80, "Padding compressed data...")
    padded_compressed = pad_to_block(compressed_data, BLOCK_SIZE, b'\x00')

    # 6. 최종 이미지 조립
    report_progress(85, "Assembling final image...")
    final_image = yaml_header + padded_compressed

    # 7. 출력 파일 경로 생성
    if output_path is None:
        date_suffix = datetime.now().strftime("%y%m%d")
        output_dir = os.path.dirname(input_path) or '.'
        output_path = os.path.join(output_dir, f"RSDK_S32DS_Flash_Image_{date_suffix}.bz2")

    # 8. 파일 저장
    report_progress(90, f"Saving to {output_path}...")
    with open(output_path, 'wb') as f:
        f.write(final_image)

    final_size = len(final_image)
    report_progress(100, f"Complete! Final size: {final_size:,} bytes")

    # 정보 딕셔너리 반환
    info = {
        'original_size': original_size,
        'compressed_size': compressed_size,
        'crc_32': crc_32,
        'crc_32_hex': f"{crc_32:08x}",
        'final_size': final_size,
        'compression_ratio': compression_ratio,
        'yaml_header_size': len(yaml_header),
        'padding_size': len(padded_compressed) - compressed_size,
        'date': date_str,
    }

    return output_path, info


def main():
    """CLI 테스트용"""
    import sys

    if len(sys.argv) < 2:
        print("Usage: python swu_flash_image_create.py <input_file> [output_file]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else None

    def progress_cb(percent, message):
        print(f"[{percent:3d}%] {message}")

    try:
        out_path, info = create_flash_image(input_path, output_path, progress_cb)
        print("\n=== Flash Image Created ===")
        print(f"Output file: {out_path}")
        print(f"Original size: {info['original_size']:,} bytes")
        print(f"Compressed size: {info['compressed_size']:,} bytes")
        print(f"CRC32: {info['crc_32_hex']}")
        print(f"Final size: {info['final_size']:,} bytes")
        print(f"Compression ratio: {info['compression_ratio']:.1f}%")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
