// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

extern crate tonic_build;

pub fn main() {
    let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let proto_files = vec![
        root.join("xronos/messages/reactor_graph.proto"),
        root.join("xronos/messages/source_info.proto"),
        root.join("xronos/services/diagram_generator.proto"),
    ];

    let descriptor_path =
        std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("proto_descriptor.bin");
    tonic_build::configure()
        .file_descriptor_set_path(&descriptor_path)
        .compile_well_known_types(true)
        .extern_path(".google.protobuf", "::pbjson_types")
        .protoc_arg("--experimental_allow_proto3_optional")
        .compile_protos(&proto_files, &[root])
        .unwrap();
    let descriptor_set = std::fs::read(&descriptor_path).unwrap();
    pbjson_build::Builder::new()
        .register_descriptors(&descriptor_set)
        .unwrap()
        .build(&["."])
        .unwrap();
}
