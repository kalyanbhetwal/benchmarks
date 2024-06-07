use proc_macro::TokenStream;
use syn::{parse_macro_input, Expr};
use quote::quote;

//mod my_proc;

#[proc_macro]
pub fn my_proc_macro(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as Expr);
    let expr_str1 = quote! {#input}.to_string();
    let expr_str = expr_str1.trim_matches('\"');

    let (first_part, second_part) = expr_str.split_at(8);

    let (f1, f2, f3): (String, String, String); 
    
    f1 = format!("{}.moder.modify(|_, w| {{w.moder{}().alternate()}});",first_part,second_part);
    let pin_no =  second_part.parse::<i32>().unwrap();
    if pin_no <=7{
        f2 = format!("{}.afrl.modify(|_, w| {{w.afrl{}().af12()}});",first_part,second_part);
    }else{
        f2 = format!("{}.afrh.modify(|_, w| {{w.afrl{}().af12()}});",first_part,second_part);
    }
    f3= format!("{}.ospeedr.modify(|_, w| {{w.ospeedr{}().very_high_speed()}});",first_part,second_part);
    let f = format!("{}{}{}", f1, f2, f3);

    // Parse the function call string into a TokenStream
    let generated_function = syn::parse_str::<proc_macro2::TokenStream>(&f)
        .expect("Failed to parse generated function");
    TokenStream::from(quote! { #generated_function })

}
