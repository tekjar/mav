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
use syn::Ident;
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
                let doc = field.text.unwrap();
                let attributes = field.attributes;
                let member = attributes["name"].clone();
                let typ = rustify(&attributes["type"].clone());

                println!("member = {}, type = {}", member, typ);
                
                let member = Ident::from(member);
                let typ = Ident::from(typ);
                fields.push(quote! {
                    #[doc = #doc]
                    #member: #typ
                });
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

fn rustify(typ: &str) -> String {
    match typ {
        "uint8_t" | "uint8_t_mavlink_version" => "u8".to_owned(),
        "uint16_t" => "u16".to_owned(),
        "uint32_t" => "u32".to_owned(),
        "uint64_t" => "u64".to_owned(),
        "int8_t"  => "i8".to_owned(),
        "int16_t" => "i16".to_owned(),
        "int32_t" => "i32".to_owned(),
        "int64_t" => "i64".to_owned(),
        //TODO: Verify float default size
        "float" => "f32".to_owned(),
        //FIXME
        _ if typ.contains("char[") => "u8".to_owned(),
        _ if typ.contains("array[") => "u8".to_owned(),
        _ if typ.contains("int8_t[") => "u8".to_owned(),
        _ => panic!("Invalid type {:?} to convert", typ)
    }
}


