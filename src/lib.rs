#[macro_use]
extern crate quote;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;
extern crate syn;
extern crate treexml;

pub mod mavlink;

use std::fs::File;
use std::io::BufReader;
use std::io::{Read, Write};
use std::path::Path;

use treexml::{Document, Element};
use serde_json::{Error, Value};
use syn::{Field, Ident};
use quote::Tokens;

#[derive(Debug, Serialize, Deserialize)]
struct MavLink {
    version: String,
}

#[derive(Debug, Serialize, Deserialize)]
struct Mav {
    mavlink: MavLink,
}

pub struct Mav2Rust {
    version: Element,
    enums: Element,
    messages: Element,
}

impl Mav2Rust {
    pub fn new(mavlink_spec: &Path) -> Mav2Rust {
        let mut schema_file = File::open(mavlink_spec).unwrap();
        let mut schema = String::new();

        let _ = schema_file.read_to_string(&mut schema).unwrap();
        let schema = schema.as_bytes();

        let tree = Document::parse(schema).unwrap();
        let mut root = tree.root.unwrap();

        let version = root.children.remove(0);
        let _dilect = root.children.remove(0);
        let enums = root.children.remove(0);
        let messages = root.children.remove(0);

        Mav2Rust {
            version,
            enums,
            messages,
        }
    }

    pub fn generate(&self, output_file: &Path) {
        let mut mavlink = File::create(output_file).unwrap();
        let mut root = vec![];
        let children = self.messages.children.clone();

        for mut message in children {
            // println!("{:?}", message);

            // break;
            let name = message.attributes.get("name").unwrap();
            let description = message.children.remove(0).text.unwrap();

            let name = Ident::from(name.as_ref());
            let mut fields = vec![];

            for field in message.children {
                // println!("{:?}", field);
                if field.name == "extensions" {
                    continue;
                }

                let doc = &field.text.unwrap();
                let attributes = field.attributes;
                let member = attributes.get("name").unwrap();
                let typ = attributes.get("type").unwrap();

                // println!("member = {}, type = {}", member, typ);
                fields.push(rustify(typ, doc, member));
            }
            let tokens = quote!(
                    #[doc = #description]
                    pub struct #name {
                        #(#fields,)*
                    }


            );

            root.push(tokens);
        }

        let tokens = quote!(#(#root)*);
        mavlink.write_all(tokens.to_string().as_bytes()).unwrap();
    }
}

fn rustify(typ: &str, doc: &str, member: &str) -> Tokens {
    // change "type" attribute name to "typ" as type is a rust keyword
    let member = if member == "type" { "typ" } else { member };

    // add dummy doc if description doesn't exist
    let doc = if doc.len() == 0 {
        println!("{}", member);
        "No documentation about this in spec"
    } else {
        doc
    };

    match typ {
        "uint8_t" | "uint8_t_mavlink_version" => {
            let member = Ident::from(member);
            let typ = Ident::from("u8");
            quote! {#[doc = #doc] #member: #typ}
        }
        "uint16_t" => {
            let member = Ident::from(member);
            let typ = Ident::from("u16");
            quote! {#[doc = #doc] #member: #typ}
        }
        "uint32_t" => {
            let member = Ident::from(member);
            let typ = Ident::from("u32");
            quote! {#[doc = #doc] #member: #typ}
        }
        "uint64_t" => {
            let member = Ident::from(member);
            let typ = Ident::from("u64");
            quote! {#[doc = #doc] #member: #typ}
        }
        "int8_t" | "uint8_t_mavlink_version" => {
            let member = Ident::from(member);
            let typ = Ident::from("i8");
            quote! {#[doc = #doc] #member: #typ}
        }
        "int16_t" => {
            let member = Ident::from(member);
            let typ = Ident::from("i16");
            quote! {#[doc = #doc] #member: #typ}
        }
        "int32_t" => {
            let member = Ident::from(member);
            let typ = Ident::from("i32");
            quote! {#[doc = #doc] #member: #typ}
        }
        "int64_t" => {
            let member = Ident::from(member);
            let typ = Ident::from("i64");
            quote! {#[doc = #doc] #member: #typ}
        }
        "float" => {
            let member = Ident::from(member);
            let typ = Ident::from("f32");
            quote! {#[doc = #doc] #member: #typ}
        }
        _ if typ.contains("char[") => {
            let member = Ident::from(member);
            let size: usize = get_array_size(typ);
            quote! {#[doc = #doc] #member: [char; #size]}
        }
        _ if typ.contains("int8_t[") => {
            let member = Ident::from(member);
            let size: usize = get_array_size(typ);
            quote! {#[doc = #doc] #member: [i8; #size]}
        }
        _ if typ.contains("int16_t[") => {
            let member = Ident::from(member);
            let size: usize = get_array_size(typ);
            quote! {#[doc = #doc] #member: [i16; #size]}
        }
        _ if typ.contains("int32_t[") => {
            let member = Ident::from(member);
            let size: usize = get_array_size(typ);
            quote! {#[doc = #doc] #member: [i32; #size]}
        }
        _ if typ.contains("float[") => {
            let member = Ident::from(member);
            let size: usize = get_array_size(typ);
            quote! {#[doc = #doc] #member: [f32; #size]}
        }
        _ => panic!("Invalid type {:?} to convert", typ),
    }
    // match typ {
    //     "uint8_t" | "uint8_t_mavlink_version" => "u8".to_owned(),
    //     "uint16_t" => "u16".to_owned(),
    //     "uint32_t" => "u32".to_owned(),
    //     "uint64_t" => "u64".to_owned(),
    //     "int8_t"  => "i8".to_owned(),
    //     "int16_t" => "i16".to_owned(),
    //     "int32_t" => "i32".to_owned(),
    //     "int64_t" => "i64".to_owned(),
    //     //TODO: Verify float default size
    //     "float" => "f32".to_owned(),
    //     _ if typ.contains("char[") => "[char; ".to_owned() + typ.get(5..6).unwrap() + "]",
    //     _ if typ.contains("int8_t[") => "[int8; ".to_owned() + typ.get(5..6).unwrap() + "]",
    //     _ => panic!("Invalid type {:?} to convert", typ)
    // }
}

fn get_array_size(array: &str) -> usize {
    let cut_left: Vec<_> = array.split("[").collect();
    let start = cut_left[1];
    let cut_right: Vec<_> = start.split("]").collect();
    let size = cut_right[0];

    return size.parse().unwrap();
}
